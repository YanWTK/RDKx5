#!/usr/bin/env python3
"""Understand a spoken command as a structured robot task."""

from __future__ import annotations

import json
import os
import re
from pathlib import Path
from typing import Any

import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import String

from .bailian_vlm_client import DEFAULT_BASE_URL as BAILIAN_DEFAULT_BASE_URL
from .direct_bailian_client import DirectBailianClient


DEFAULT_TASK_PROMPT = """你是名叫“小瓜”的家庭服务机器人的语音任务理解与任务规划器。
你的任务是把用户的话转换成严格 JSON，供机器人路由到正确执行流程。

只允许输出一个 JSON 对象，不要输出 Markdown，不要解释。

JSON 字段必须包含：
{
  "intent": "fetch_to_speaker" | "come_to_speaker" | "navigate_to" | "task_chain" | "transfer_object" | "deliver_to_person" | "chat" | "unknown",
  "target_name": "用户最终想要的完整目标描述；如果有空间关系、位置修饰、参照物修饰，必须完整保留，例如 除锈剂旁边的矿泉水、桌子左边的杯子、最右边的饮料；如果没有明确物品则为空字符串",
  "semantic_hint": "主物品类别或语义线索，只写真正要抓取的主物品和可检测类别，例如 矿泉水 饮料 瓶装饮料 bottle、杯子 cup、遥控器 remote、书 book；不要包含旁边、左边、右边、最近等空间关系；没有则为空字符串",
  "source_location": "物品起点或第一个地点；没有则为空字符串",
  "destination_location": "目标地点；拿给说话人时填 speaker；没有则为空字符串",
  "search_locations": ["地点名列表；当记忆库找不到目标时，机器人应该现场搜索的候选地点；由你根据用户原话、明确地点和家庭常识判断；只写地点名，没有则为空数组"],
  "delivery_target": "speaker 或明确地点名；没有则为空字符串",
  "placement_reference": "相对放置任务的参照物，例如 矿泉水；没有则为空字符串",
  "placement_side": "left | right | front | back | none；表示把 target_name 放到参照物哪一侧；没有则为 none",
  "need_sound_localization": true 或 false,
  "need_grasp": true 或 false,
  "need_return_to_speaker": true 或 false,
  "release_at_destination": true 或 false,
  "tasks": [
    {"action": "record_speaker | navigate | go_to_object_memory | detect_and_grasp | return_to_speaker | release | chat", "location": "地点名，可空", "target_name": "物品名或完整目标描述，可空"}
  ],
  "plan": [
    {"action": "record_speaker | navigate_to | find_object | find_person | grasp_object | place_relative | release_object | return_to_speaker | say | ask_user | chat", "location": "地点名，可空", "target": "目标物品名或人物描述，可空", "reference": "放置参照物，可空", "side": "left|right|front|back|none", "search_locations": ["候选搜索地点，可空"]}
  ],
  "tts_text": "一句很短的中文确认话"
}

判断规则：
1. 只有用户明确要求“拿给我、递给我、给我带来、我渴了给我拿喝的”等把物品交给说话人的请求，intent 才是 fetch_to_speaker。
   - 此类必须 need_sound_localization=true、need_grasp=true、need_return_to_speaker=true、destination_location=speaker、delivery_target=speaker。
   - 这是唯一需要声源定位和记录说话人位置的任务。
   - 如果用户说“去/到/在/从/把A的 某物 拿给我”，A 是物品所在地点，必须填 source_location=A；不能因为 destination_location=speaker 而丢掉 source_location。
   - 如果用户说“带着/拿着/送到/回到 某个明确地点”，例如“回到原点”“送到客厅”，这不是拿给说话人；必须 need_sound_localization=false、need_return_to_speaker=false，plan 里使用 navigate_to(location)，不要使用 return_to_speaker。
2. 用户只是让机器人去某个位置，intent 为 navigate_to。
   - 不声源定位，不抓取，need_sound_localization=false、need_grasp=false。
3. 用户说“过来一下、来我这里、到我这边、过来找我、来我身边”等让机器人移动到说话人附近的请求，intent 为 come_to_speaker。
   - 需要声源定位和记录说话人位置，但不抓取物品：need_sound_localization=true、need_grasp=false、need_return_to_speaker=false、destination_location=speaker、delivery_target=speaker。
   - tasks 只包含 record_speaker 和 navigate，navigate 的 location 填 speaker。
   - plan 包含 record_speaker 和 navigate_to speaker。
   - 这类任务不允许生成 go_to_object_memory、detect_and_grasp 或 return_to_speaker。
4. 用户要求按顺序去多个位置，intent 为 task_chain。
   - tasks 里按顺序放多个 navigate 动作，不声源定位，不抓取。
   - task_chain 只用于纯导航顺序任务，例如“先去卧室再去客厅”。
5. 用户要求把一个物品从 A 移动到 B，intent 为 transfer_object。
   - 不声源定位；source_location=A，destination_location=B，need_grasp=true，need_return_to_speaker=false。
   - tasks 顺序为 navigate(A)、detect_and_grasp(物品)、navigate(B)，如果用户明确要求放下/放到/送到则 release_at_destination=true 并添加 release。
   - 如果 B 不是地点，而是“参照物左边/右边/前面/后面”，例如“把快递放到矿泉水左边”，仍然是 transfer_object，但：
     placement_reference 必须填参照物“矿泉水”，placement_side 填 left/right/front/back。
     target_name 只写真正要抓取搬运的物品，例如“快递”，不要写成“矿泉水左边的快递”。
     如果用户明确说“放到/放在 某地点 的 参照物 左边/右边/前面/后面”，destination_location 必须填该地点。
     只有没有明确地点时，destination_location 才留空；delivery_target 填参照物名，release_at_destination=true。
     tasks 顺序为 go_to_object_memory(快递)、detect_and_grasp(快递)、go_to_object_memory(矿泉水)、release(矿泉水)。
     plan 必须显式包含 find_object(要搬运物品)、grasp_object、navigate_to(参照物地点，如果有)、find_object(放置参照物)、place_relative、release_object。
     不能只在顶层 placement_reference/placement_side 里保留参照物而省略 plan 里的 place_relative；顶层字段只是兼容旧执行器，plan 才是完整执行依据。
6. 用户要求把物品拿给/递给/送给某个非说话人的人物目标，例如“正在拖地的人、穿红衣服的人、客厅里的人”，intent 为 deliver_to_person。
   - 这不是 fetch_to_speaker，不做声源定位，need_sound_localization=false、need_return_to_speaker=false。
   - source_location 填物品所在地点；delivery_target 填完整人物描述，例如“正在拖地的人”；destination_location 留空，除非用户明确给出人物所在地点。
   - need_grasp=true，release_at_destination=false；机器人抓到物品后会搜索并导航到该人物附近，默认保持夹持。
   - plan 必须包含 find_object(物品)、grasp_object、find_person(人物描述)、navigate_to(person)。
7. 闲聊、问答、陪聊、天气、讲故事、介绍自己等不需要机器人移动或抓取的请求，intent 为 chat。
   - 不执行物理动作，tasks 为空或只含 chat。
   - 如果用户问你是谁、叫什么、介绍自己，tts_text 要明确回答“我是机器人小瓜”，可以再补一句很短的能力说明。
   - 如果用户问“X在哪、你知道X在哪、X在什么位置、你记得X在哪吗”等物品位置查询，仍然是 chat，不执行物理动作；但必须提取 target_name=X、semantic_hint=物品类别线索，tts_text 写成“我帮你查一下X的位置”或“我查一下记忆库”。
   - 物品位置查询允许后续记忆库做近似匹配：如果没有完全匹配，可以选择大概相关但不确定的物品，例如用户找“卡皮巴拉玩偶”，记忆库只有“毛绒玩具”，后续可以播报“记忆里没有完全匹配卡皮巴拉玩偶，但毛绒玩具可能在卧室”。
8. 无法确定地点、物品或物理动作风险较高时，intent 为 unknown，不要擅自执行物理动作。
9. 地点名称尽量使用用户原话，例如 卧室、客厅、原点、厨房、A、B。
   - search_locations 是“记忆库查不到时”的现场搜索候选地点，由你根据用户原话和语义自行分析，不要机械固定某个物品一定在哪个房间。
   - 如果用户明确指定了物品所在地点，例如“去卧室拿矿泉水”，search_locations 只填该地点。
   - 如果用户没有明确地点，例如“帮我拿一瓶水”，必须从“当前可搜索地点”里按家庭常识给出 1 到 3 个最可能地点；不要留空数组，除非当前没有可搜索地点或完全不涉及找物品。
   - search_locations 只能使用“当前可搜索地点”中的 name，不要输出别名、point_id、speaker 或原点。
   - 相对放置任务中，要抓取的物品按 source_location 推断 search_locations；参照物所在地点按 destination_location 理解。
10. “去完A再去B”“先去A然后去B”一定是 task_chain，不是 fetch_to_speaker。
11. “把A的东西/把某物从A拿到B”一定是 transfer_object，不是 fetch_to_speaker。
12. 如果用户中途改口或纠正，例如“还是、算了、不对、改成、改去、那就、额还是”，后半句是最终意图；不要把改口前后的两个地点识别成 task_chain。
13. 如果用户说“X旁边的Y”“X左边的Y”“X右边的Y”“靠近X的Y”“离X最近的Y”“最左边的Y”“最右边的Y”“前面的Y”“后面的Y”，target_name 必须保留完整目标描述，不能只保留主物品。
   - 例如“除锈剂旁边的矿泉水”，target_name 必须是“除锈剂旁边的矿泉水”，不能写成“矿泉水”。
   - 例如“桌子左边的杯子”，target_name 必须是“桌子左边的杯子”，不能写成“杯子”。
   - 例如“最右边那瓶水”，target_name 必须是“最右边的水”或“最右边那瓶水”，不能写成“水”。
   - semantic_hint 只写真正要抓取的主物品和类别，例如“矿泉水 饮料 瓶装饮料 bottle”，不要写“除锈剂旁边”。
14. 参照物不是地点时，不要把它填入 source_location。
   - 例如“除锈剂旁边的矿泉水”中，除锈剂是参照物，不是 source_location。
   - 例如“遥控器旁边的手机”中，遥控器是参照物，不是 source_location。
15. tasks 中的 go_to_object_memory 和 detect_and_grasp 的 target_name 应与顶层 target_name 保持一致，必须保留完整目标描述。
   - 这样后面的记忆选择、视觉确认和 VLM 目标选择都能看到完整需求。
16. tts_text 可以自然一点，但不要丢掉关键修饰关系。
   - 例如用户要“除锈剂旁边的矿泉水”，tts_text 应该说“好的，我去拿除锈剂旁边的矿泉水给你”。
17. “把A放到B左边/右边/旁边”这类是搬运 A，不是抓 B；B 是放置参照物，不是要抓取的目标。
   - 如果 A 自身带有空间关系修饰，例如“最靠近卡比巴拉玩偶的那个农夫山泉矿泉水”，target_name、tasks.target_name、plan 中 find_object/grasp_object/place_relative/release_object 的 target 都必须完整保留这段描述，不能缩短成“农夫山泉矿泉水”或“矿泉水”。
   - 如果 B 自身带有品牌、类别或地点修饰，例如“客厅的双汇鸭舌箱柠檬茶”，placement_reference 和 plan 中 reference/find_object target 必须完整保留“双汇鸭舌箱柠檬茶”，不能只写“柠檬茶”或“矿泉水”。
18. plan 是未来执行器使用的完整步骤列表，必须尽量完整表达用户原话；tasks 是当前兼容旧执行器的简化字段。
   - 所有需要机器人动作的任务都要输出 plan；纯聊天 plan 可为空或只含 chat。
   - plan 只能使用列出的 action，不能发明动作名。
   - find_object 表示导航到记忆点或现场搜索目标；location 是明确地点，search_locations 是记忆找不到时的候选地点。
   - grasp_object 表示对准并抓取上一步找到的目标。
   - place_relative 表示把当前抓着的物体放到 reference 的 side 方位，reference 不是要抓取的目标。
   - return_to_speaker 只能用于“拿给我、递给我、带给我”等交给说话人的任务；如果目的地是明确地点，例如原点、卧室、客厅，必须使用 navigate_to。
   - “回到原点/去原点/回原点/再回到原点”默认只是机器人自身导航到原点；该步骤只能生成 navigate_to，target 必须为空字符串，不能生成 find_object 或 grasp_object。
   - 不要把上一句里的放置参照物继承成“回到原点”的目标物品。例如“放到客厅矿泉水左边，然后回到原点”中，“客厅矿泉水”只是参照物，不是要拿回原点的物品。
   - 只有用户明确说“带着X回到原点/拿着X回到原点/把X带回原点”时，才可以为了 X 生成 find_object、grasp_object，再 navigate_to 原点；否则不能新增抓取动作。
   - 如果一句话包含两个及以上物理子任务，例如“先把A放到B右边，然后把C拿给我”，当前旧执行器不能安全执行，intent 必须设为 unknown，tasks 必须为空，但 plan 必须完整拆出所有步骤，tts_text 说明“任务较复杂，我先确认计划”。
   - 不要因为复杂任务而把它降级成纯导航 task_chain；不要编造“原点”步骤，除非用户明确要求去原点或回原点。

示例：
用户：我渴了，给我哪一瓶冰红茶
输出：{"intent":"fetch_to_speaker","target_name":"冰红茶","semantic_hint":"饮料 瓶装饮料 bottle","source_location":"","destination_location":"speaker","search_locations":["卧室","客厅"],"delivery_target":"speaker","need_sound_localization":true,"need_grasp":true,"need_return_to_speaker":true,"release_at_destination":false,"tasks":[{"action":"record_speaker","location":"","target_name":""},{"action":"go_to_object_memory","location":"","target_name":"冰红茶"},{"action":"detect_and_grasp","location":"","target_name":"冰红茶"},{"action":"return_to_speaker","location":"speaker","target_name":"冰红茶"}],"tts_text":"好的，我去拿冰红茶给你"}

用户：帮我拿一瓶水
输出：{"intent":"fetch_to_speaker","target_name":"水","semantic_hint":"饮料 瓶装饮料 bottle","source_location":"","destination_location":"speaker","search_locations":["卧室","客厅"],"delivery_target":"speaker","need_sound_localization":true,"need_grasp":true,"need_return_to_speaker":true,"release_at_destination":false,"tasks":[{"action":"record_speaker","location":"","target_name":""},{"action":"go_to_object_memory","location":"","target_name":"水"},{"action":"detect_and_grasp","location":"","target_name":"水"},{"action":"return_to_speaker","location":"speaker","target_name":"水"}],"tts_text":"好的，我去拿水给你"}

用户：帮我拿一个在除锈剂旁边的矿泉水
输出：{"intent":"fetch_to_speaker","target_name":"除锈剂旁边的矿泉水","semantic_hint":"矿泉水 饮料 瓶装饮料 bottle","source_location":"","destination_location":"speaker","delivery_target":"speaker","need_sound_localization":true,"need_grasp":true,"need_return_to_speaker":true,"release_at_destination":false,"tasks":[{"action":"record_speaker","location":"","target_name":""},{"action":"go_to_object_memory","location":"","target_name":"除锈剂旁边的矿泉水"},{"action":"detect_and_grasp","location":"","target_name":"除锈剂旁边的矿泉水"},{"action":"return_to_speaker","location":"speaker","target_name":"除锈剂旁边的矿泉水"}],"tts_text":"好的，我去拿除锈剂旁边的矿泉水给你"}

用户：帮我拿遥控器旁边的手机
输出：{"intent":"fetch_to_speaker","target_name":"遥控器旁边的手机","semantic_hint":"手机 cell phone","source_location":"","destination_location":"speaker","delivery_target":"speaker","need_sound_localization":true,"need_grasp":true,"need_return_to_speaker":true,"release_at_destination":false,"tasks":[{"action":"record_speaker","location":"","target_name":""},{"action":"go_to_object_memory","location":"","target_name":"遥控器旁边的手机"},{"action":"detect_and_grasp","location":"","target_name":"遥控器旁边的手机"},{"action":"return_to_speaker","location":"speaker","target_name":"遥控器旁边的手机"}],"tts_text":"好的，我去拿遥控器旁边的手机给你"}

用户：帮我拿最右边那瓶水
输出：{"intent":"fetch_to_speaker","target_name":"最右边那瓶水","semantic_hint":"水 饮料 瓶装饮料 bottle","source_location":"","destination_location":"speaker","delivery_target":"speaker","need_sound_localization":true,"need_grasp":true,"need_return_to_speaker":true,"release_at_destination":false,"tasks":[{"action":"record_speaker","location":"","target_name":""},{"action":"go_to_object_memory","location":"","target_name":"最右边那瓶水"},{"action":"detect_and_grasp","location":"","target_name":"最右边那瓶水"},{"action":"return_to_speaker","location":"speaker","target_name":"最右边那瓶水"}],"tts_text":"好的，我去拿最右边那瓶水给你"}

用户：去卧室把矿泉水拿给我
输出：{"intent":"fetch_to_speaker","target_name":"矿泉水","semantic_hint":"矿泉水 饮料 瓶装饮料 bottle","source_location":"卧室","destination_location":"speaker","search_locations":["卧室"],"delivery_target":"speaker","need_sound_localization":true,"need_grasp":true,"need_return_to_speaker":true,"release_at_destination":false,"tasks":[{"action":"record_speaker","location":"","target_name":""},{"action":"go_to_object_memory","location":"卧室","target_name":"矿泉水"},{"action":"detect_and_grasp","location":"卧室","target_name":"矿泉水"},{"action":"return_to_speaker","location":"speaker","target_name":"矿泉水"}],"tts_text":"好的，我去卧室拿矿泉水给你"}

用户：去卧室拿除锈剂旁边的矿泉水给我
输出：{"intent":"fetch_to_speaker","target_name":"除锈剂旁边的矿泉水","semantic_hint":"矿泉水 饮料 瓶装饮料 bottle","source_location":"卧室","destination_location":"speaker","delivery_target":"speaker","need_sound_localization":true,"need_grasp":true,"need_return_to_speaker":true,"release_at_destination":false,"tasks":[{"action":"record_speaker","location":"","target_name":""},{"action":"go_to_object_memory","location":"卧室","target_name":"除锈剂旁边的矿泉水"},{"action":"detect_and_grasp","location":"卧室","target_name":"除锈剂旁边的矿泉水"},{"action":"return_to_speaker","location":"speaker","target_name":"除锈剂旁边的矿泉水"}],"tts_text":"好的，我去卧室拿除锈剂旁边的矿泉水给你"}

用户：去卧室
输出：{"intent":"navigate_to","target_name":"","semantic_hint":"","source_location":"","destination_location":"卧室","delivery_target":"卧室","need_sound_localization":false,"need_grasp":false,"need_return_to_speaker":false,"release_at_destination":false,"tasks":[{"action":"navigate","location":"卧室","target_name":""}],"tts_text":"好的，我去卧室"}

用户：过来一下
输出：{"intent":"come_to_speaker","target_name":"","semantic_hint":"","source_location":"","destination_location":"speaker","delivery_target":"speaker","need_sound_localization":true,"need_grasp":false,"need_return_to_speaker":false,"release_at_destination":false,"tasks":[{"action":"record_speaker","location":"","target_name":""},{"action":"navigate","location":"speaker","target_name":""}],"plan":[{"action":"record_speaker","location":"","target":"","reference":"","side":"none","search_locations":[]},{"action":"navigate_to","location":"speaker","target":"","reference":"","side":"none","search_locations":[]}],"tts_text":"好的，我过来"}

用户：先去卧室再去客厅
输出：{"intent":"task_chain","target_name":"","semantic_hint":"","source_location":"","destination_location":"","delivery_target":"","need_sound_localization":false,"need_grasp":false,"need_return_to_speaker":false,"release_at_destination":false,"tasks":[{"action":"navigate","location":"卧室","target_name":""},{"action":"navigate","location":"客厅","target_name":""}],"tts_text":"好的，我先去卧室，再去客厅"}

用户：帮我去客厅，还是去卧室吧，拿一个水杯给我
输出：{"intent":"fetch_to_speaker","target_name":"水杯","semantic_hint":"杯子 cup","source_location":"卧室","destination_location":"speaker","delivery_target":"speaker","need_sound_localization":true,"need_grasp":true,"need_return_to_speaker":true,"release_at_destination":false,"tasks":[{"action":"record_speaker","location":"","target_name":""},{"action":"go_to_object_memory","location":"卧室","target_name":"水杯"},{"action":"detect_and_grasp","location":"卧室","target_name":"水杯"},{"action":"return_to_speaker","location":"speaker","target_name":"水杯"}],"tts_text":"好的，我去卧室拿水杯给你"}

用户：把卧室的水杯送到客厅
输出：{"intent":"transfer_object","target_name":"水杯","semantic_hint":"杯子 cup","source_location":"卧室","destination_location":"客厅","delivery_target":"客厅","need_sound_localization":false,"need_grasp":true,"need_return_to_speaker":false,"release_at_destination":true,"tasks":[{"action":"navigate","location":"卧室","target_name":""},{"action":"detect_and_grasp","location":"卧室","target_name":"水杯"},{"action":"navigate","location":"客厅","target_name":"水杯"},{"action":"release","location":"客厅","target_name":"水杯"}],"tts_text":"好的，我把卧室的水杯送到客厅"}

用户：帮忙把卧室的水拿给正在拖地的人
输出：{"intent":"deliver_to_person","target_name":"水","semantic_hint":"水 饮料 瓶装饮料 bottle","source_location":"卧室","destination_location":"","search_locations":["卧室"],"delivery_target":"正在拖地的人","placement_reference":"","placement_side":"none","need_sound_localization":false,"need_grasp":true,"need_return_to_speaker":false,"release_at_destination":false,"tasks":[{"action":"go_to_object_memory","location":"卧室","target_name":"水"},{"action":"detect_and_grasp","location":"卧室","target_name":"水"},{"action":"navigate","location":"person","target_name":"正在拖地的人"}],"plan":[{"action":"find_object","location":"卧室","target":"水","reference":"","side":"none","search_locations":["卧室"]},{"action":"grasp_object","location":"卧室","target":"水","reference":"","side":"none","search_locations":[]},{"action":"find_person","location":"","target":"正在拖地的人","reference":"","side":"none","search_locations":[]},{"action":"navigate_to","location":"person","target":"正在拖地的人","reference":"","side":"none","search_locations":[]}],"tts_text":"好的，我去卧室拿水给正在拖地的人"}

用户：把卧室里桌子左边的杯子送到客厅
输出：{"intent":"transfer_object","target_name":"桌子左边的杯子","semantic_hint":"杯子 cup","source_location":"卧室","destination_location":"客厅","delivery_target":"客厅","need_sound_localization":false,"need_grasp":true,"need_return_to_speaker":false,"release_at_destination":true,"tasks":[{"action":"navigate","location":"卧室","target_name":""},{"action":"detect_and_grasp","location":"卧室","target_name":"桌子左边的杯子"},{"action":"navigate","location":"客厅","target_name":"桌子左边的杯子"},{"action":"release","location":"客厅","target_name":"桌子左边的杯子"}],"tts_text":"好的，我把卧室里桌子左边的杯子送到客厅"}

用户：帮我把快递放到矿泉水左边
输出：{"intent":"transfer_object","target_name":"快递","semantic_hint":"快递 包裹 package box","source_location":"","destination_location":"","delivery_target":"矿泉水","placement_reference":"矿泉水","placement_side":"left","need_sound_localization":false,"need_grasp":true,"need_return_to_speaker":false,"release_at_destination":true,"tasks":[{"action":"go_to_object_memory","location":"","target_name":"快递"},{"action":"detect_and_grasp","location":"","target_name":"快递"},{"action":"go_to_object_memory","location":"","target_name":"矿泉水"},{"action":"release","location":"","target_name":"矿泉水"}],"tts_text":"好的，我先去拿快递，再把它放到矿泉水左边"}

用户：把包裹放在矿泉水右边
输出：{"intent":"transfer_object","target_name":"包裹","semantic_hint":"快递 包裹 package box","source_location":"","destination_location":"","delivery_target":"矿泉水","placement_reference":"矿泉水","placement_side":"right","need_sound_localization":false,"need_grasp":true,"need_return_to_speaker":false,"release_at_destination":true,"tasks":[{"action":"go_to_object_memory","location":"","target_name":"包裹"},{"action":"detect_and_grasp","location":"","target_name":"包裹"},{"action":"go_to_object_memory","location":"","target_name":"矿泉水"},{"action":"release","location":"","target_name":"矿泉水"}],"tts_text":"好的，我把包裹放到矿泉水右边"}

用户：帮忙把卧室最靠近卡比巴拉玩偶的那个农夫山泉矿泉水搬到客厅的双汇鸭舌箱柠檬茶的右边
输出：{"intent":"transfer_object","target_name":"最靠近卡比巴拉玩偶的那个农夫山泉矿泉水","semantic_hint":"农夫山泉矿泉水 饮料 瓶装饮料 bottle","source_location":"卧室","destination_location":"客厅","search_locations":["卧室","客厅"],"delivery_target":"客厅","placement_reference":"双汇鸭舌箱柠檬茶","placement_side":"right","need_sound_localization":false,"need_grasp":true,"need_return_to_speaker":false,"release_at_destination":true,"tasks":[{"action":"navigate","location":"卧室","target_name":""},{"action":"detect_and_grasp","location":"卧室","target_name":"最靠近卡比巴拉玩偶的那个农夫山泉矿泉水"},{"action":"navigate","location":"客厅","target_name":"最靠近卡比巴拉玩偶的那个农夫山泉矿泉水"},{"action":"release","location":"客厅","target_name":"双汇鸭舌箱柠檬茶"}],"plan":[{"action":"find_object","location":"卧室","target":"最靠近卡比巴拉玩偶的那个农夫山泉矿泉水","reference":"","side":"none","search_locations":["卧室"]},{"action":"grasp_object","location":"卧室","target":"最靠近卡比巴拉玩偶的那个农夫山泉矿泉水","reference":"","side":"none","search_locations":[]},{"action":"navigate_to","location":"客厅","target":"","reference":"","side":"none","search_locations":[]},{"action":"find_object","location":"客厅","target":"双汇鸭舌箱柠檬茶","reference":"","side":"none","search_locations":["客厅"]},{"action":"place_relative","location":"客厅","target":"最靠近卡比巴拉玩偶的那个农夫山泉矿泉水","reference":"双汇鸭舌箱柠檬茶","side":"right","search_locations":[]},{"action":"release_object","location":"客厅","target":"最靠近卡比巴拉玩偶的那个农夫山泉矿泉水","reference":"","side":"none","search_locations":[]}],"tts_text":"好的，我去卧室拿最靠近卡比巴拉玩偶的那瓶水，放到客厅双汇鸭舌箱柠檬茶右边"}

用户：帮我把卧室的农夫山泉矿泉水放到卧室的矿泉水的右边，然后把卧室左侧矿泉水拿给我
输出：{"intent":"unknown","target_name":"左侧矿泉水","semantic_hint":"矿泉水 饮料 瓶装饮料 bottle","source_location":"卧室","destination_location":"speaker","search_locations":["卧室"],"delivery_target":"speaker","placement_reference":"矿泉水","placement_side":"right","need_sound_localization":true,"need_grasp":true,"need_return_to_speaker":true,"release_at_destination":true,"tasks":[],"plan":[{"action":"record_speaker","location":"","target":"","reference":"","side":"none","search_locations":[]},{"action":"find_object","location":"卧室","target":"农夫山泉矿泉水","reference":"","side":"none","search_locations":["卧室"]},{"action":"grasp_object","location":"卧室","target":"农夫山泉矿泉水","reference":"","side":"none","search_locations":[]},{"action":"find_object","location":"卧室","target":"矿泉水","reference":"","side":"none","search_locations":["卧室"]},{"action":"place_relative","location":"卧室","target":"农夫山泉矿泉水","reference":"矿泉水","side":"right","search_locations":[]},{"action":"release_object","location":"卧室","target":"农夫山泉矿泉水","reference":"","side":"none","search_locations":[]},{"action":"find_object","location":"卧室","target":"左侧矿泉水","reference":"","side":"none","search_locations":["卧室"]},{"action":"grasp_object","location":"卧室","target":"左侧矿泉水","reference":"","side":"none","search_locations":[]},{"action":"return_to_speaker","location":"speaker","target":"左侧矿泉水","reference":"","side":"none","search_locations":[]}],"tts_text":"这个任务较复杂，我先确认计划喵"}

用户：帮我把卧室的农夫山泉矿泉水放到客厅的矿泉水的左边，然后再回到原点
输出：{"intent":"unknown","target_name":"农夫山泉矿泉水","semantic_hint":"矿泉水 饮料 瓶装饮料 bottle","source_location":"卧室","destination_location":"原点","search_locations":["卧室","客厅"],"delivery_target":"原点","placement_reference":"矿泉水","placement_side":"left","need_sound_localization":false,"need_grasp":true,"need_return_to_speaker":false,"release_at_destination":true,"tasks":[],"plan":[{"action":"find_object","location":"卧室","target":"农夫山泉矿泉水","reference":"","side":"none","search_locations":["卧室"]},{"action":"grasp_object","location":"卧室","target":"农夫山泉矿泉水","reference":"","side":"none","search_locations":[]},{"action":"navigate_to","location":"客厅","target":"","reference":"","side":"none","search_locations":[]},{"action":"find_object","location":"客厅","target":"矿泉水","reference":"","side":"none","search_locations":["客厅"]},{"action":"place_relative","location":"客厅","target":"农夫山泉矿泉水","reference":"矿泉水","side":"left","search_locations":[]},{"action":"release_object","location":"客厅","target":"农夫山泉矿泉水","reference":"","side":"none","search_locations":[]},{"action":"navigate_to","location":"原点","target":"","reference":"","side":"none","search_locations":[]}],"tts_text":"这个任务较复杂，我先确认计划喵"}

用户：先把卧室的农夫山泉矿泉水放到客厅矿泉水的左边，然后带着客厅右侧的矿泉水回到原点
输出：{"intent":"unknown","target_name":"客厅右侧的矿泉水","semantic_hint":"矿泉水 饮料 瓶装饮料 bottle","source_location":"卧室","destination_location":"原点","search_locations":["卧室","客厅"],"delivery_target":"原点","placement_reference":"矿泉水","placement_side":"left","need_sound_localization":false,"need_grasp":true,"need_return_to_speaker":false,"release_at_destination":true,"tasks":[],"plan":[{"action":"find_object","location":"卧室","target":"农夫山泉矿泉水","reference":"","side":"none","search_locations":["卧室"]},{"action":"grasp_object","location":"卧室","target":"农夫山泉矿泉水","reference":"","side":"none","search_locations":[]},{"action":"navigate_to","location":"客厅","target":"","reference":"","side":"none","search_locations":[]},{"action":"find_object","location":"客厅","target":"矿泉水","reference":"","side":"none","search_locations":["客厅"]},{"action":"place_relative","location":"客厅","target":"农夫山泉矿泉水","reference":"矿泉水","side":"left","search_locations":[]},{"action":"release_object","location":"客厅","target":"农夫山泉矿泉水","reference":"","side":"none","search_locations":[]},{"action":"find_object","location":"客厅","target":"客厅右侧的矿泉水","reference":"","side":"none","search_locations":["客厅"]},{"action":"grasp_object","location":"客厅","target":"客厅右侧的矿泉水","reference":"","side":"none","search_locations":[]},{"action":"navigate_to","location":"原点","target":"客厅右侧的矿泉水","reference":"","side":"none","search_locations":[]}],"tts_text":"这个任务较复杂，我先确认计划喵"}

用户：今天天气怎么样
输出：{"intent":"chat","target_name":"","semantic_hint":"","source_location":"","destination_location":"","delivery_target":"","need_sound_localization":false,"need_grasp":false,"need_return_to_speaker":false,"release_at_destination":false,"tasks":[],"tts_text":"可以，我陪你聊聊"}

用户：你是谁
输出：{"intent":"chat","target_name":"","semantic_hint":"","source_location":"","destination_location":"","delivery_target":"","need_sound_localization":false,"need_grasp":false,"need_return_to_speaker":false,"release_at_destination":false,"tasks":[],"tts_text":"我是机器人小瓜，可以帮你找东西和搬东西"}

用户：你知道卡皮巴拉玩偶在哪吗
输出：{"intent":"chat","target_name":"卡皮巴拉玩偶","semantic_hint":"玩偶 毛绒玩具 teddy bear","source_location":"","destination_location":"","delivery_target":"","need_sound_localization":false,"need_grasp":false,"need_return_to_speaker":false,"release_at_destination":false,"tasks":[],"plan":[{"action":"chat","location":"","target":"卡皮巴拉玩偶","reference":"","side":"none","search_locations":[]}],"tts_text":"我帮你查一下卡皮巴拉玩偶的位置"}

用户：{user_command}
输出："""



class TaskUnderstandingNode(Node):
    def __init__(self) -> None:
        super().__init__("task_understanding")

        self.declare_parameter("query_topic", "/task_understanding/query")
        self.declare_parameter("result_topic", "/task_understanding/result")
        self.declare_parameter("use_local_llm", True)
        self.declare_parameter("llm_url", "http://127.0.0.1:8000/analyze")
        self.declare_parameter("request_timeout_sec", 20.0)
        self.declare_parameter("prompt_template", DEFAULT_TASK_PROMPT)
        self.declare_parameter("bailian_model", "qwen3.6-flash")
        self.declare_parameter("bailian_base_url", "")
        self.declare_parameter("bailian_api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("bailian_enable_thinking", False)
        self.declare_parameter(
            "location_config_path",
            os.getenv(
                "VOICE_TASK_LOCATION_CONFIG",
                "/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/patrol_points.json",
            ),
        )
        self.declare_parameter(
            "speech_profile_path",
            os.getenv(
                "VOICE_PERSONA_PATH",
                "/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/voice_persona.json",
            ),
        )
        self.declare_parameter("speech_profile", os.getenv("VOICE_PERSONA_PROFILE", ""))

        self._query_topic = str(self.get_parameter("query_topic").value)
        self._result_topic = str(self.get_parameter("result_topic").value)
        self._use_local_llm = _as_bool(self.get_parameter("use_local_llm").value)
        self._llm_url = str(self.get_parameter("llm_url").value)
        self._timeout = float(self.get_parameter("request_timeout_sec").value)
        self._prompt_template = str(self.get_parameter("prompt_template").value)
        self._bailian_model = str(self.get_parameter("bailian_model").value)
        self._bailian_base_url = (
            str(self.get_parameter("bailian_base_url").value).strip()
            or BAILIAN_DEFAULT_BASE_URL
        )
        self._bailian_api_key_env = str(self.get_parameter("bailian_api_key_env").value)
        self._bailian_enable_thinking = _as_bool(
            self.get_parameter("bailian_enable_thinking").value
        )
        self._location_config_path = Path(
            str(self.get_parameter("location_config_path").value)
        ).expanduser()
        self._location_config_mtime = None
        self._location_context = ""
        self._speech_profile_path = Path(
            str(self.get_parameter("speech_profile_path").value)
        ).expanduser()
        self._speech_profile_name = str(self.get_parameter("speech_profile").value).strip()
        self._speech_profile_mtime = None
        self._speech_profile = {}
        self._bailian_client = None

        self._result_pub = self.create_publisher(String, self._result_topic, 10)
        self.create_subscription(String, self._query_topic, self._on_query, 10)

        self.get_logger().info(
            "task_understanding started | "
            f"query={self._query_topic} result={self._result_topic} "
            f"mode={'local' if self._use_local_llm else 'bailian'}"
        )

    def _on_query(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
            if not isinstance(cmd, dict):
                raise ValueError("query payload must be an object")
        except Exception as exc:
            self._publish({"success": False, "reason": f"invalid JSON: {exc}"})
            return

        request_id = str(cmd.get("request_id") or "").strip()
        user_command = str(cmd.get("user_command") or cmd.get("query") or "").strip()
        recent_context = str(cmd.get("recent_context") or "").strip()
        if not user_command:
            self._publish({
                "success": False,
                "request_id": request_id,
                "reason": "user_command is empty",
            })
            return

        try:
            context_parts = [
                part
                for part in (self._active_location_context(), _format_recent_context(recent_context))
                if part
            ]
            prompt = _render_task_prompt(
                self._prompt_template,
                user_command,
                "\n".join(context_parts),
            )
            prompt = self._apply_speech_profile_to_prompt(prompt)
            raw_reply = self._ask_local_llm(prompt) if self._use_local_llm else self._ask_bailian(prompt)
            parsed = _parse_json_object(raw_reply)
            if parsed is None:
                raise ValueError(f"LLM did not return valid JSON: {raw_reply[:200]!r}")
            result = _normalize_result(parsed)
            result = _apply_command_location_hints(result, user_command)
            result, validation_meta = self._repair_plan_once(
                user_command=user_command,
                result=result,
            )
            result.update({
                "success": True,
                "request_id": request_id,
                "user_command": user_command,
                "raw_reply": raw_reply,
                "plan_validation": validation_meta,
            })
            self._publish(result)
        except Exception as exc:
            fallback = _heuristic_understand(user_command)
            fallback.setdefault("search_locations", [])
            fallback.setdefault("plan", [])
            fallback.update({
                "success": False,
                "request_id": request_id,
                "user_command": user_command,
                "reason": str(exc),
            })
            self._publish(fallback)

    def _active_location_context(self) -> str:
        path = self._location_config_path
        try:
            mtime = path.stat().st_mtime
        except OSError:
            self._location_config_mtime = None
            self._location_context = ""
            return ""

        if self._location_config_mtime == mtime:
            return self._location_context

        try:
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            points = data.get("points", []) if isinstance(data, dict) else []
            nav_locations = []
            search_locations = []
            for point in points:
                if not isinstance(point, dict):
                    continue
                name = str(point.get("name") or point.get("point_id") or "").strip()
                if not name:
                    continue
                aliases = point.get("aliases", [])
                if not isinstance(aliases, list):
                    aliases = []
                entry = {
                    "name": name,
                    "point_id": str(point.get("point_id") or "").strip(),
                    "aliases": [str(item).strip() for item in aliases if str(item).strip()],
                }
                nav_locations.append(entry)
                if _as_bool(point.get("scan", True)):
                    search_locations.append(name)
            if nav_locations:
                context = (
                    "\n当前可导航地点 JSON：\n"
                    f"{json.dumps(nav_locations, ensure_ascii=False)}\n"
                    "当前可搜索地点："
                    f"{json.dumps(search_locations, ensure_ascii=False)}\n"
                    "地点规则：source_location、destination_location、tasks.location 只能优先使用当前可导航地点的 name；"
                    "search_locations 只能使用当前可搜索地点中的 name。"
                )
            else:
                context = ""
            self._location_context = context
            self._location_config_mtime = mtime
            if context:
                self.get_logger().info(f"loaded task locations from {path}: {len(nav_locations)}")
        except Exception as exc:
            self.get_logger().warn(f"failed to load task locations {path}: {exc}")
            self._location_context = ""
            self._location_config_mtime = mtime
        return self._location_context

    def _ask_local_llm(self, prompt: str) -> str:
        response = requests.post(
            self._llm_url,
            json={"prompt": prompt},
            timeout=self._timeout,
        )
        if response.status_code != 200:
            raise RuntimeError(f"local LLM HTTP {response.status_code}: {response.text[:160]}")
        body = response.json()
        return str(body.get("ai_response") or body.get("response") or body.get("text") or "")

    def _ask_bailian(self, prompt: str) -> str:
        if self._bailian_client is None:
            api_key = os.getenv(self._bailian_api_key_env, "").strip()
            if not api_key:
                raise RuntimeError(f"missing {self._bailian_api_key_env}")
            self._bailian_client = DirectBailianClient(
                api_key=api_key,
                base_url=self._bailian_base_url,
                timeout=self._timeout,
                component="task_understanding",
                log_callback=self.get_logger().info,
            )
        return self._bailian_client.chat_completion(
            model=self._bailian_model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0,
            # DashScope OpenAI-compatible endpoint requires nesting under
            # chat_template_kwargs for enable_thinking to take effect.
            extra_body={"chat_template_kwargs": {"enable_thinking": self._bailian_enable_thinking}},
            payload_kind="utf8_prompt",
            payload_bytes=len(prompt.encode("utf-8")),
        )

    def _ask_task_llm(self, prompt: str) -> str:
        return self._ask_local_llm(prompt) if self._use_local_llm else self._ask_bailian(prompt)

    def _repair_plan_once(
        self,
        *,
        user_command: str,
        result: dict[str, Any],
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        issues = _validate_plan_consistency(result)
        if not issues:
            return result, {
                "checked": True,
                "ok": True,
                "repaired": False,
                "issues": [],
            }

        self.get_logger().warn(
            "task plan consistency issues, requesting one repair: "
            + json.dumps(issues, ensure_ascii=False)
        )
        meta = {
            "checked": True,
            "ok": False,
            "repaired": False,
            "issues": issues,
            "repair_attempted": True,
        }
        repair_prompt = _render_plan_repair_prompt(user_command, result, issues)
        try:
            repaired_raw = self._ask_task_llm(repair_prompt)
            repaired_parsed = _parse_json_object(repaired_raw)
            if repaired_parsed is None:
                raise ValueError(f"repair LLM did not return valid JSON: {repaired_raw[:200]!r}")
            repaired = _normalize_result(repaired_parsed)
            repaired = _apply_command_location_hints(repaired, user_command)
            repaired_issues = _validate_plan_consistency(repaired)
            meta.update({
                "repair_raw_reply": repaired_raw,
                "repair_issues": repaired_issues,
                "ok": not repaired_issues,
                "repaired": True,
            })
            if repaired_issues:
                self.get_logger().warn(
                    "task plan repair still has consistency issues; using repaired result once: "
                    + json.dumps(repaired_issues, ensure_ascii=False)
                )
            else:
                self.get_logger().info("task plan repaired by one-shot consistency check")
            return repaired, meta
        except Exception as exc:
            meta["repair_error"] = str(exc)
            self.get_logger().warn(
                f"task plan one-shot repair failed; using original result: {exc}"
            )
            return result, meta

    def _publish(self, payload: dict[str, Any]) -> None:
        text = json.dumps(payload, ensure_ascii=False)
        self.get_logger().info(text)
        self._result_pub.publish(String(data=text))

    def _active_speech_profile(self) -> dict[str, Any]:
        path = self._speech_profile_path
        try:
            mtime = path.stat().st_mtime
        except OSError:
            self._speech_profile = {}
            self._speech_profile_mtime = None
            return {}

        if self._speech_profile_mtime == mtime:
            return self._speech_profile

        try:
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            profiles = data.get("profiles", {}) if isinstance(data, dict) else {}
            requested = self._speech_profile_name or str(data.get("active_profile") or "default")
            profile = profiles.get(requested) or profiles.get("default") or {}
            if not isinstance(profile, dict):
                profile = {}
            self._speech_profile = profile
            self._speech_profile_mtime = mtime
            self.get_logger().info(f"speech profile loaded: {requested} from {path}")
        except Exception as exc:
            self.get_logger().warn(f"failed to load speech profile {path}: {exc}")
            self._speech_profile = {}
            self._speech_profile_mtime = mtime
        return self._speech_profile

    def _apply_speech_profile_to_prompt(self, prompt: str) -> str:
        profile = self._active_speech_profile()
        instruction = ""
        if isinstance(profile, dict):
            instruction = str(profile.get("task_tts_instruction") or "").strip()
        if not instruction:
            return prompt

        block = (
            "\n播报口气要求：\n"
            f"{instruction}\n"
            "只影响 JSON 里的 tts_text 字段，不要改变 intent、target_name、semantic_hint 的判断。\n"
        )
        marker = "\n用户："
        index = prompt.rfind(marker)
        if index >= 0:
            return prompt[:index] + block + prompt[index:]
        return prompt + block


def _render_task_prompt(template: str, user_command: str, location_context: str = "") -> str:
    """Insert the command without interpreting JSON braces in the prompt."""
    context = str(location_context or "").strip()
    if context:
        marker = "\n用户："
        index = template.rfind(marker)
        if index >= 0:
            template = template[:index] + "\n" + context + "\n" + template[index:]
        else:
            template = f"{template.rstrip()}\n{context}\n"
    placeholder = "{user_command}"
    if placeholder in template:
        return template.replace(placeholder, user_command)
    return f"{template.rstrip()}\n\n用户：{user_command}\n输出："


def _format_recent_context(context: str) -> str:
    text = str(context or "").strip()
    if not text:
        return ""
    return (
        "\n最近上下文：\n"
        f"{text}\n"
        "上下文规则：最近上下文只用于理解“它、刚才、再试一次、换一个、为什么失败”等省略表达；"
        "如果用户当前明确说了新的地点、物品或动作，必须以当前指令为准。"
    )


def _render_plan_repair_prompt(
    user_command: str,
    result: dict[str, Any],
    issues: list[str],
) -> str:
    return (
        "你是家庭服务机器人“小瓜”的任务 plan 一致性修正器。\n"
        "你会收到用户原话、上一轮 JSON 和一致性问题。\n"
        "请只修正 JSON 中不自洽的字段，尤其是 plan；不要改变用户原话语义，不要新增用户没说过的物品、地点或动作。\n"
        "只允许输出一个 JSON 对象，不要输出 Markdown，不要解释。\n"
        "plan 允许的 action 只有：record_speaker, navigate_to, find_object, grasp_object, "
        "place_relative, release_object, return_to_speaker, say, ask_user, chat。\n"
        "place_relative 表示把当前抓着的目标放到 reference 的 side 方位；reference 是放置参照物，不是要抓取的目标。\n"
        "如果有 placement_reference 和 placement_side，plan 必须包含 find_object(reference)、place_relative(reference, side) 和 release_object。\n"
        "如果要抓取 target_name，grasp_object 前必须先有 find_object(target_name)。\n"
        "如果目的地是明确地点，plan 中必须有 navigate_to(location)；如果 intent 是 fetch_to_speaker 且目的地是 speaker，则使用 return_to_speaker；如果 intent 是 come_to_speaker，则使用 navigate_to(speaker)。\n\n"
        f"用户原话：{user_command}\n\n"
        "一致性问题：\n"
        f"{json.dumps(issues, ensure_ascii=False, indent=2)}\n\n"
        "上一轮 JSON：\n"
        f"{json.dumps(result, ensure_ascii=False, indent=2)}\n\n"
        "修正后的 JSON："
    )


def _parse_json_object(text: str) -> dict[str, Any] | None:
    raw = str(text or "").strip()
    if raw.startswith("```"):
        raw = re.sub(r"^```(?:json)?", "", raw, flags=re.IGNORECASE).strip()
        raw = re.sub(r"```$", "", raw).strip()
    match = re.search(r"\{.*\}", raw, flags=re.DOTALL)
    if match:
        raw = match.group(0)
    try:
        data = json.loads(raw)
    except Exception:
        return None
    return data if isinstance(data, dict) else None


def _validate_plan_consistency(result: dict[str, Any]) -> list[str]:
    if not isinstance(result, dict):
        return ["result is not a JSON object"]
    plan = result.get("plan")
    if not isinstance(plan, list):
        return ["plan must be a list"]
    issues: list[str] = []
    actions = [_plan_action(step) for step in plan]

    target = str(result.get("target_name") or "").strip()
    destination = str(result.get("destination_location") or "").strip()
    placement_reference = str(result.get("placement_reference") or "").strip()
    placement_side = _normalize_placement_side(result.get("placement_side"))
    need_grasp = _as_bool(result.get("need_grasp", False))
    release_at_destination = _as_bool(result.get("release_at_destination", False))
    intent = str(result.get("intent") or "").strip()

    if placement_reference and placement_side != "none":
        if not _plan_has_action(plan, "place_relative"):
            issues.append(
                "placement_reference/placement_side are set but plan has no place_relative step"
            )
        elif not _plan_has_place_relative(plan, placement_reference, placement_side):
            issues.append(
                "plan place_relative does not preserve placement_reference and placement_side"
            )
        if not _plan_has_find_object(plan, placement_reference):
            issues.append("relative placement plan should find_object the placement reference")
        if release_at_destination and not _plan_has_action(plan, "release_object"):
            issues.append("release_at_destination is true but plan has no release_object step")

    if destination and destination not in ("speaker", "none"):
        if intent in ("navigate_to", "task_chain", "transfer_object", "deliver_to_person", "unknown"):
            if not _plan_has_navigate_to(plan, destination):
                issues.append("destination_location is set but plan has no matching navigate_to step")
    if destination == "speaker" and result.get("need_return_to_speaker") is True:
        if not _plan_has_action(plan, "return_to_speaker"):
            issues.append("need_return_to_speaker is true but plan has no return_to_speaker step")

    if need_grasp and target:
        grasp_indices = [
            index for index, step in enumerate(plan)
            if _plan_action(step) == "grasp_object"
        ]
        if not grasp_indices:
            issues.append("need_grasp is true but plan has no grasp_object step")
        else:
            first_grasp = grasp_indices[0]
            if not _plan_has_find_object(plan[:first_grasp], target):
                issues.append("grasp_object appears before any find_object for target_name")

    if actions.count("release_object") and not (
        "grasp_object" in actions or "place_relative" in actions
    ):
        issues.append("plan has release_object without prior grasp/place context")

    return issues


def _plan_action(step: Any) -> str:
    if not isinstance(step, dict):
        return ""
    return str(step.get("action") or "").strip()


def _plan_has_action(plan: list[Any], action: str) -> bool:
    return any(_plan_action(step) == action for step in plan)


def _plan_text_matches(value: Any, expected: str) -> bool:
    value_text = str(value or "").strip()
    expected_text = str(expected or "").strip()
    if not value_text or not expected_text:
        return False
    return (
        value_text == expected_text
        or value_text in expected_text
        or expected_text in value_text
    )


def _plan_has_find_object(plan: list[Any], target: str) -> bool:
    for step in plan:
        if not isinstance(step, dict) or _plan_action(step) != "find_object":
            continue
        if _plan_text_matches(step.get("target"), target):
            return True
    return False


def _plan_has_navigate_to(plan: list[Any], location: str) -> bool:
    for step in plan:
        if not isinstance(step, dict) or _plan_action(step) != "navigate_to":
            continue
        if _plan_text_matches(step.get("location"), location):
            return True
    return False


def _plan_has_place_relative(plan: list[Any], reference: str, side: str) -> bool:
    normalized_side = _normalize_placement_side(side)
    for step in plan:
        if not isinstance(step, dict) or _plan_action(step) != "place_relative":
            continue
        if (
            _plan_text_matches(step.get("reference"), reference)
            and _normalize_placement_side(step.get("side")) == normalized_side
        ):
            return True
    return False


def _normalize_result(data: dict[str, Any]) -> dict[str, Any]:
    intent = str(data.get("intent") or "unknown").strip()
    if intent == "fetch_object":
        intent = "fetch_to_speaker"
    allowed = (
        "fetch_to_speaker",
        "come_to_speaker",
        "navigate_to",
        "task_chain",
        "transfer_object",
        "deliver_to_person",
        "chat",
        "unknown",
    )
    if intent not in allowed:
        intent = "unknown"
    target_name = str(data.get("target_name") or "").strip()
    semantic_hint = str(data.get("semantic_hint") or "").strip()
    source_location = str(data.get("source_location") or "").strip()
    destination_location = str(data.get("destination_location") or "").strip()
    search_locations = _normalize_string_list(data.get("search_locations"))
    delivery_target = str(data.get("delivery_target") or "").strip()
    placement_reference = str(data.get("placement_reference") or "").strip()
    placement_side = _normalize_placement_side(data.get("placement_side"))
    if not delivery_target:
        if intent in ("fetch_to_speaker", "come_to_speaker"):
            delivery_target = "speaker"
        elif placement_reference:
            delivery_target = placement_reference
        else:
            delivery_target = destination_location
    if intent == "come_to_speaker" and not destination_location:
        destination_location = "speaker"
    need_sound = _as_bool(data.get("need_sound_localization", intent in ("fetch_to_speaker", "come_to_speaker")))
    need_grasp = _as_bool(data.get("need_grasp", intent in ("fetch_to_speaker", "transfer_object", "deliver_to_person")))
    need_return = _as_bool(data.get("need_return_to_speaker", intent == "fetch_to_speaker"))
    release_at_destination = _as_bool(data.get("release_at_destination", False))
    tasks = _normalize_tasks(data.get("tasks"))
    if not tasks:
        tasks = _default_tasks(
            intent,
            target_name,
            source_location,
            destination_location,
            release_at_destination,
            delivery_target=delivery_target,
        )
    plan = _normalize_plan(data.get("plan"))
    if not plan:
        plan = _default_plan_from_tasks(tasks)
    tts_text = str(data.get("tts_text") or "").strip()
    if not tts_text:
        if intent == "fetch_to_speaker":
            tts_text = f"好的，我去拿{target_name or semantic_hint or '目标'}给你"
        elif intent == "come_to_speaker":
            tts_text = "好的，我过来"
        elif intent == "navigate_to":
            tts_text = f"好的，我去{destination_location or '目标位置'}"
        elif intent == "task_chain":
            names = [task.get("location") for task in tasks if task.get("action") == "navigate"]
            tts_text = "好的，我按顺序过去" if not names else f"好的，我按顺序去{'、'.join(names)}"
        elif intent == "transfer_object":
            tts_text = f"好的，我把{source_location or '指定位置'}的{target_name or '物品'}送到{destination_location or '目标位置'}"
        elif intent == "deliver_to_person":
            tts_text = f"好的，我去{source_location or '指定位置'}拿{target_name or '物品'}给{delivery_target or '那个人'}"
        elif intent == "chat":
            tts_text = "可以，我陪你聊聊"
        else:
            tts_text = "我还不确定该执行什么任务"
    return {
        "intent": intent,
        "target_name": target_name,
        "semantic_hint": semantic_hint,
        "source_location": source_location,
        "destination_location": destination_location,
        "search_locations": search_locations,
        "delivery_target": delivery_target,
        "placement_reference": placement_reference,
        "placement_side": placement_side,
        "need_sound_localization": need_sound,
        "need_grasp": need_grasp,
        "need_return_to_speaker": need_return,
        "release_at_destination": release_at_destination,
        "tasks": tasks,
        "plan": plan,
        "tts_text": tts_text,
    }


def _apply_command_location_hints(result: dict[str, Any], user_command: str) -> dict[str, Any]:
    """Recover explicit source locations that the LLM sometimes drops."""
    if result.get("intent") == "transfer_object" and str(result.get("placement_reference") or "").strip():
        result = _apply_relative_placement_location_hint(result, user_command)

    if result.get("intent") == "deliver_to_person":
        return _apply_deliver_to_person_hints(result, user_command)

    if result.get("intent") != "fetch_to_speaker":
        return result
    if str(result.get("source_location") or "").strip():
        return result

    location = _extract_source_location(user_command)
    if not location:
        return result

    result = dict(result)
    result["source_location"] = location
    tasks = []
    for task in result.get("tasks") or []:
        if not isinstance(task, dict):
            continue
        task = dict(task)
        if task.get("action") in ("go_to_object_memory", "detect_and_grasp"):
            task["location"] = location
        tasks.append(task)
    if not tasks:
        tasks = _default_tasks(
            "fetch_to_speaker",
            str(result.get("target_name") or ""),
            location,
            "speaker",
            False,
        )
    result["tasks"] = tasks
    target = str(result.get("target_name") or result.get("semantic_hint") or "目标").strip()
    result["tts_text"] = f"好的，我去{location}拿{target}给你"
    return result


def _apply_relative_placement_location_hint(result: dict[str, Any], user_command: str) -> dict[str, Any]:
    if str(result.get("destination_location") or "").strip():
        return result
    reference = str(result.get("placement_reference") or "").strip()
    location = _extract_relative_placement_location(user_command, reference)
    if not location:
        return result

    result = dict(result)
    result["destination_location"] = location
    tasks = []
    for task in result.get("tasks") or []:
        if not isinstance(task, dict):
            continue
        task = dict(task)
        if (
            task.get("action") in ("go_to_object_memory", "release")
            and str(task.get("target_name") or "").strip() == reference
        ):
            task["location"] = location
        tasks.append(task)
    result["tasks"] = tasks
    return result


def _apply_deliver_to_person_hints(result: dict[str, Any], user_command: str) -> dict[str, Any]:
    result = dict(result)
    if not str(result.get("source_location") or "").strip():
        location = _extract_object_source_location(user_command)
        if location:
            result["source_location"] = location
    delivery_target = str(result.get("delivery_target") or "").strip()
    if not delivery_target:
        delivery_target = _extract_person_delivery_target(user_command)
        if delivery_target:
            result["delivery_target"] = delivery_target
    result["need_sound_localization"] = False
    result["need_return_to_speaker"] = False
    result["need_grasp"] = True
    result["release_at_destination"] = False
    if not result.get("tasks"):
        result["tasks"] = _default_tasks(
            "deliver_to_person",
            str(result.get("target_name") or ""),
            str(result.get("source_location") or ""),
            "",
            False,
            delivery_target=str(result.get("delivery_target") or ""),
        )
    return result


def _extract_relative_placement_location(text: str, reference: str = "") -> str:
    known_locations = ("卧室", "客厅", "原点", "厨房", "书房", "餐厅", "卫生间", "阳台", "A", "B", "a", "b")
    command = str(text or "")
    reference = str(reference or "").strip()
    for loc in known_locations:
        if loc not in command:
            continue
        patterns = []
        for verb in ("放到", "放在", "摆到", "摆在"):
            patterns.extend((
                f"{verb}{loc}的",
                f"{verb}{loc}",
            ))
            if reference:
                patterns.extend((
                    f"{verb}{loc}的{reference}",
                    f"{verb}{loc}{reference}",
                ))
        if any(pattern in command for pattern in patterns):
            return loc
    return ""


def _extract_object_source_location(text: str) -> str:
    known_locations = ("卧室", "客厅", "原点", "厨房", "书房", "餐厅", "卫生间", "阳台", "A", "B", "a", "b")
    command = str(text or "")
    for loc in known_locations:
        if loc not in command:
            continue
        if any(pattern in command for pattern in (f"{loc}的", f"从{loc}", f"去{loc}", f"到{loc}")):
            return loc
    return ""


def _extract_source_location(text: str) -> str:
    known_locations = ("卧室", "客厅", "原点", "厨房", "书房", "餐厅", "卫生间", "阳台", "A", "B", "a", "b")
    command = str(text or "")
    if not any(term in command for term in ("拿给我", "递给我", "带给我", "给我")):
        return ""
    for loc in known_locations:
        if loc not in command:
            continue
        patterns = (
            f"去{loc}",
            f"到{loc}",
            f"在{loc}",
            f"从{loc}",
            f"{loc}的",
            f"把{loc}的",
        )
        if any(pattern in command for pattern in patterns):
            return loc
    return ""


def _extract_person_delivery_target(text: str) -> str:
    command = str(text or "")
    for marker in ("拿给", "递给", "送给", "带给"):
        if marker not in command:
            continue
        tail = command.split(marker, 1)[-1].strip(" ，。,.")
        return tail
    return ""


def _heuristic_understand(user_command: str) -> dict[str, Any]:
    text = user_command.strip()
    beverage_terms = ("渴", "喝", "饮料", "水", "茶", "冰红茶", "绿茶", "可乐", "咖啡", "奶")
    fetch_to_speaker_terms = ("给我", "拿给我", "递给我", "带给我")
    navigate_terms = ("去", "到", "前往")
    come_to_speaker_terms = ("过来", "来我这里", "来我这", "到我这里", "到我这边", "来我身边", "过来找我")
    known_locations = ("卧室", "客厅", "原点", "厨房", "A", "B", "a", "b")
    person_delivery = _extract_person_delivery_target(text)
    if person_delivery and person_delivery != "我" and "人" in person_delivery:
        source = _extract_object_source_location(text)
        target = "物品"
        if source and f"{source}的" in text:
            after_source = text.split(f"{source}的", 1)[-1]
            for marker in ("拿给", "递给", "送给", "带给"):
                if marker in after_source:
                    target = after_source.split(marker, 1)[0].strip() or target
                    break
        elif "水" in text:
            target = "水"
        return {
            "intent": "deliver_to_person",
            "target_name": target,
            "semantic_hint": _semantic_hint_for_target(target),
            "source_location": source,
            "destination_location": "",
            "delivery_target": person_delivery,
            "placement_reference": "",
            "placement_side": "none",
            "need_sound_localization": False,
            "need_grasp": True,
            "need_return_to_speaker": False,
            "release_at_destination": False,
            "tasks": _default_tasks("deliver_to_person", target, source, "", False, delivery_target=person_delivery),
            "tts_text": f"好的，我去{source or '指定位置'}拿{target}给{person_delivery}",
        }
    relative = _extract_relative_placement(text)
    if relative:
        target, reference, side = relative
        return {
            "intent": "transfer_object",
            "target_name": target,
            "semantic_hint": _semantic_hint_for_target(target),
            "source_location": "",
            "destination_location": "",
            "delivery_target": reference,
            "placement_reference": reference,
            "placement_side": side,
            "need_sound_localization": False,
            "need_grasp": True,
            "need_return_to_speaker": False,
            "release_at_destination": True,
            "tasks": _relative_placement_tasks(target, reference),
            "tts_text": f"好的，我先去拿{target}，再把它放到{reference}{_side_text(side)}",
        }
    if any(term in text for term in beverage_terms) and (
        any(term in text for term in fetch_to_speaker_terms) or "渴" in text
    ):
        target = "饮料"
        for candidate in ("矿泉水", "瓶装水", "冰红茶", "绿茶", "可乐", "咖啡", "牛奶", "水"):
            if candidate in text:
                target = candidate
                break
        source = _extract_source_location(text)
        return {
            "intent": "fetch_to_speaker",
            "target_name": target,
            "semantic_hint": "饮料 瓶装饮料",
            "source_location": source,
            "destination_location": "speaker",
            "delivery_target": "speaker",
            "placement_reference": "",
            "placement_side": "none",
            "need_sound_localization": True,
            "need_grasp": True,
            "need_return_to_speaker": True,
            "release_at_destination": False,
            "tasks": _default_tasks("fetch_to_speaker", target, source, "speaker", False),
            "tts_text": f"好的，我去{source}拿{target}给你" if source else f"好的，我去拿{target}给你",
        }
    if any(term in text for term in come_to_speaker_terms):
        return {
            "intent": "come_to_speaker",
            "target_name": "",
            "semantic_hint": "",
            "source_location": "",
            "destination_location": "speaker",
            "delivery_target": "speaker",
            "placement_reference": "",
            "placement_side": "none",
            "need_sound_localization": True,
            "need_grasp": False,
            "need_return_to_speaker": False,
            "release_at_destination": False,
            "tasks": _default_tasks("come_to_speaker", "", "", "speaker", False),
            "tts_text": "好的，我过来",
        }
    if ("从" in text or "的" in text) and any(term in text for term in ("送到", "拿到", "放到", "搬到")):
        locations = [loc for loc in known_locations if loc in text]
        source = locations[0] if locations else ""
        dest = locations[-1] if len(locations) > 1 else ""
        target = text
        if source and "的" in text:
            after_source = text.split(source, 1)[-1]
            if after_source.startswith("的"):
                after_source = after_source[1:]
            for marker in ("送到", "拿到", "放到", "搬到"):
                if marker in after_source:
                    target = after_source.split(marker, 1)[0].strip() or target
                    break
        return {
            "intent": "transfer_object",
            "target_name": target,
            "semantic_hint": "",
            "source_location": source,
            "destination_location": dest,
            "delivery_target": dest,
            "placement_reference": "",
            "placement_side": "none",
            "need_sound_localization": False,
            "need_grasp": True,
            "need_return_to_speaker": False,
            "release_at_destination": any(term in text for term in ("放到", "放下", "送到")),
            "tasks": _default_tasks("transfer_object", target, source, dest, any(term in text for term in ("放到", "放下", "送到"))),
            "tts_text": "好的，我去搬运这个物品",
        }
    if any(term in text for term in ("再去", "然后去", "再到", "然后到", "先去")):
        locations = [loc for loc in known_locations if loc in text]
        return {
            "intent": "task_chain",
            "target_name": "",
            "semantic_hint": "",
            "source_location": "",
            "destination_location": "",
            "delivery_target": "",
            "placement_reference": "",
            "placement_side": "none",
            "need_sound_localization": False,
            "need_grasp": False,
            "need_return_to_speaker": False,
            "release_at_destination": False,
            "tasks": [{"action": "navigate", "location": loc, "target_name": ""} for loc in locations],
            "tts_text": "好的，我按顺序过去",
        }
    if any(term in text for term in navigate_terms) and any(loc in text for loc in known_locations):
        loc = next((loc for loc in known_locations if loc in text), "")
        return {
            "intent": "navigate_to",
            "target_name": "",
            "semantic_hint": "",
            "source_location": "",
            "destination_location": loc,
            "delivery_target": loc,
            "placement_reference": "",
            "placement_side": "none",
            "need_sound_localization": False,
            "need_grasp": False,
            "need_return_to_speaker": False,
            "release_at_destination": False,
            "tasks": [{"action": "navigate", "location": loc, "target_name": ""}],
            "tts_text": f"好的，我去{loc}",
        }
    return {
        "intent": "chat",
        "target_name": "",
        "semantic_hint": "",
        "source_location": "",
        "destination_location": "",
        "delivery_target": "",
        "placement_reference": "",
        "placement_side": "none",
        "need_sound_localization": False,
        "need_grasp": False,
        "need_return_to_speaker": False,
        "release_at_destination": False,
        "tasks": [],
        "tts_text": "可以，我陪你聊聊",
    }


def _normalize_tasks(value: Any) -> list[dict[str, str]]:
    if not isinstance(value, list):
        return []
    tasks: list[dict[str, str]] = []
    allowed_actions = {
        "record_speaker",
        "navigate",
        "go_to_object_memory",
        "detect_and_grasp",
        "return_to_speaker",
        "release",
        "chat",
    }
    for item in value:
        if not isinstance(item, dict):
            continue
        action = str(item.get("action") or "").strip()
        if action not in allowed_actions:
            continue
        tasks.append({
            "action": action,
            "location": str(item.get("location") or "").strip(),
            "target_name": str(item.get("target_name") or "").strip(),
        })
    return tasks


def _normalize_plan(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        return []
    allowed_actions = {
        "record_speaker",
        "navigate_to",
        "find_object",
        "find_person",
        "grasp_object",
        "place_relative",
        "release_object",
        "return_to_speaker",
        "say",
        "ask_user",
        "chat",
    }
    plan: list[dict[str, Any]] = []
    for item in value:
        if not isinstance(item, dict):
            continue
        action = str(item.get("action") or "").strip()
        if action not in allowed_actions:
            continue
        plan.append({
            "action": action,
            "location": str(item.get("location") or "").strip(),
            "target": str(item.get("target") or item.get("target_name") or "").strip(),
            "reference": str(item.get("reference") or "").strip(),
            "side": _normalize_placement_side(item.get("side")),
            "search_locations": _normalize_string_list(item.get("search_locations")),
        })
    return plan


def _default_plan_from_tasks(tasks: list[dict[str, str]]) -> list[dict[str, Any]]:
    action_map = {
        "record_speaker": "record_speaker",
        "navigate": "navigate_to",
        "go_to_object_memory": "find_object",
        "detect_and_grasp": "grasp_object",
        "return_to_speaker": "return_to_speaker",
        "release": "release_object",
        "chat": "chat",
    }
    plan = []
    for task in tasks or []:
        action = action_map.get(str(task.get("action") or "").strip())
        if not action:
            continue
        location = str(task.get("location") or "").strip()
        target = str(task.get("target_name") or "").strip()
        plan.append({
            "action": action,
            "location": location,
            "target": target,
            "reference": "",
            "side": "none",
            "search_locations": [location] if action == "find_object" and location else [],
        })
    return plan


def _normalize_string_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        items = [value]
    elif isinstance(value, (list, tuple)):
        items = value
    else:
        return []
    result = []
    seen = set()
    for item in items:
        text = str(item or "").strip()
        if not text or text in seen:
            continue
        seen.add(text)
        result.append(text)
    return result


def _normalize_placement_side(value: Any) -> str:
    text = str(value or "").strip().lower()
    mapping = {
        "左": "left",
        "左边": "left",
        "左侧": "left",
        "left": "left",
        "右": "right",
        "右边": "right",
        "右侧": "right",
        "right": "right",
        "前": "front",
        "前面": "front",
        "front": "front",
        "后": "back",
        "后面": "back",
        "back": "back",
    }
    return mapping.get(text, "none")


def _side_text(side: str) -> str:
    return {
        "left": "左边",
        "right": "右边",
        "front": "前面",
        "back": "后面",
    }.get(side, "旁边")


def _semantic_hint_for_target(target: str) -> str:
    text = str(target or "")
    if any(term in text for term in ("快递", "包裹", "盒", "箱")):
        return "快递 包裹 盒子 package box"
    if any(term in text for term in ("矿泉水", "水", "饮料", "茶", "可乐")):
        return "饮料 瓶装饮料 bottle"
    if any(term in text for term in ("杯", "水杯")):
        return "杯子 cup"
    return ""


def _extract_relative_placement(text: str) -> tuple[str, str, str] | None:
    command = str(text or "").strip()
    if not command:
        return None

    side_patterns = (
        ("left", ("左边", "左侧", "左面")),
        ("right", ("右边", "右侧", "右面")),
        ("front", ("前面", "前边")),
        ("back", ("后面", "后边")),
    )
    side = ""
    side_word = ""
    for candidate_side, words in side_patterns:
        for word in words:
            if word in command:
                side = candidate_side
                side_word = word
                break
        if side:
            break
    if not side or not any(term in command for term in ("放到", "放在", "摆到", "摆在", "放")):
        return None

    pattern = (
        r"(?:帮我|请|麻烦你)?(?:把|将)?(?P<target>.+?)"
        r"(?:放到|放在|摆到|摆在|放)"
        r"(?P<reference>.+?)"
        + re.escape(side_word)
    )
    match = re.search(pattern, command)
    if not match:
        return None

    target = _clean_object_phrase(match.group("target"))
    reference = _clean_object_phrase(match.group("reference"))
    if not target or not reference:
        return None
    return target, reference, side


def _clean_object_phrase(value: str) -> str:
    text = str(value or "").strip()
    changed = True
    while changed:
        changed = False
        for prefix in ("帮我", "请", "麻烦你", "把", "将", "这个", "那个", "一个", "一件", "一瓶"):
            if text.startswith(prefix):
                text = text[len(prefix):].strip()
                changed = True
    for suffix in ("一下", "吧", "呢", "啊"):
        if text.endswith(suffix):
            text = text[: -len(suffix)].strip()
    return text


def _relative_placement_tasks(target: str, reference: str) -> list[dict[str, str]]:
    return [
        {"action": "go_to_object_memory", "location": "", "target_name": target},
        {"action": "detect_and_grasp", "location": "", "target_name": target},
        {"action": "go_to_object_memory", "location": "", "target_name": reference},
        {"action": "release", "location": "", "target_name": reference},
    ]


def _default_tasks(
    intent: str,
    target_name: str,
    source_location: str,
    destination_location: str,
    release_at_destination: bool,
    delivery_target: str = "",
) -> list[dict[str, str]]:
    if intent == "fetch_to_speaker":
        return [
            {"action": "record_speaker", "location": "", "target_name": ""},
            {"action": "go_to_object_memory", "location": source_location, "target_name": target_name},
            {"action": "detect_and_grasp", "location": source_location, "target_name": target_name},
            {"action": "return_to_speaker", "location": "speaker", "target_name": target_name},
        ]
    if intent == "come_to_speaker":
        return [
            {"action": "record_speaker", "location": "", "target_name": ""},
            {"action": "navigate", "location": "speaker", "target_name": ""},
        ]
    if intent == "navigate_to":
        return [{"action": "navigate", "location": destination_location, "target_name": ""}]
    if intent == "transfer_object":
        tasks = [
            {"action": "navigate", "location": source_location, "target_name": ""},
            {"action": "detect_and_grasp", "location": source_location, "target_name": target_name},
            {"action": "navigate", "location": destination_location, "target_name": target_name},
        ]
        if release_at_destination:
            tasks.append({"action": "release", "location": destination_location, "target_name": target_name})
        return tasks
    if intent == "deliver_to_person":
        return [
            {"action": "go_to_object_memory", "location": source_location, "target_name": target_name},
            {"action": "detect_and_grasp", "location": source_location, "target_name": target_name},
            {"action": "navigate", "location": "person", "target_name": delivery_target},
        ]
    if intent == "chat":
        return []
    return []


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskUnderstandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
