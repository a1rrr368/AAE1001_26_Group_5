<h1 align="center">AAE1001 Group 5 Project</h1>
<p align="center">Aircraft Route Planning and Operating Costs · 2026/27 Semester 1</p>

## Project Overview

<!-- 这里是首页简介。每题用一句话，让读者快速看懂项目范围。 -->

Group 5 studies aircraft routing and operating costs through six tasks.

- **Task 1** finds the fastest route with A* and compares aircraft costs across three scenarios.
- **Task 2** investigates where to place a jet stream to reduce operating costs.
- **Task 3** compares aircraft capacities and engine requirements for Scenario 1.
- **Task A1** plans a route through two supply checkpoints.
- **Task A2** plans a route in a randomly generated environment.
- **Task A3** compares A*, Dijkstra, and RRT*.

<details open>
<summary><strong>Table of Contents</strong></summary>

1. [Project Overview](#project-overview)
2. [Task 1](#task-1)
3. [Task 2](#task-2)
4. [Task 3](#task-3)
5. [Task A1](#task-a1)
6. [Task A2](#task-a2)
7. [Task A3](#task-a3)
8. [How to Run](#how-to-run)
9. [Team Members and Contributions](#team-members-and-contributions)
10. [References](#references)
11. [Individual Reflections](#individual-reflections)

</details>

<!-- 目录链接与下方标题对应。若改标题，记得同步修改目录中的 # 跳转链接。 -->

## Task 1

### A* Route Planning and Aircraft Selection

A* searches an eight-direction grid from (5, 40) to (20, 30). The route avoids obstacles and accounts for the 15% and 30% cost increases in the two designated regions. For each passenger scenario, the aircraft comparison screens out options that exceed the flight limit and ranks the remaining options by total operating cost.

<!-- 这里放 Task 1 路线图。下面先用了仓库已有的旧路线图。用正式脚本重新出图后，把新图放进仓库并替换路径。图上标出起终点、障碍和两个成本区。 -->

![Original Task 1 route](./Original%20Task%201%20Path)

<!-- 从正式输出填入三个场景的最终结果。总成本是整个场景的费用。表后用一句话说明胜出机型，并列出超出班次上限的机型。 -->

<!-- UPDATE T1: verified scenario results and current model assumptions. | README line 48. -->

| Scenario | Passengers | Flight limit | Fuel price ($/kg) | Selected aircraft | Flights | Total cost ($) |
|---|---:|---:|---:|---|---:|---:|
| 1 | 3,300 | 13 | 0.85 | A330-900neo | 11 | 81,751.58 |
| 2 | 1,500 | 7 | 0.96 | A330-900neo | 5 | 41,639.42 |
| 3 | 2,250 | 25 | 0.78 | A330-900neo | 8 | 53,868.54 |

The current `Task 1_Final` returns a weighted route time of **58.7875 minutes**. A330-900neo has the lowest total cost in all three scenarios. A321neo requires 17 flights in Scenario 1 and 8 flights in Scenario 2, exceedI contributed to Task 3 by checking the minimum feasible passenger capacity. I also prepared the PowerPoint slides introducing our aircraft, Doeing 7700, edited its introduction, and supplemented the README to explain our design reasoning. The scenario required us to transport 3,300 passengers within a maximum of 13 flights. Dividing 3,300 by 13 gives approximately 254 passengers per flight. Since the permitted capacities increase in steps of 50, 300 seats was the smallest feasible option. A 250-seat aircraft would require 14 flights and exceed the limit. This check helped the team focus its cost comparisons on the 300-, 350-, 400- and 450-seat designs.

Identifying this minimum capacity also established the engine configuration. Under the assignment rules, aircraft with at least 300 seats must use four engines. Therefore, every feasible design required four engines, which determined the fuel consumption rate and fixed cost used in our comparisons. Preparing the introduction slides helped me practise presenting these connected decisions clearly.

Through this assignment, I learned to check constraints before comparing costs. I came to understand that capacity, flight frequency and operating costs must be considered together. I also found that a small calculation could provide useful direction for the whole team. Explaining our work through the PowerPoint and README strengthened my confidence in communicating numerical reasoning. In future group work, I would document the requirements and assumptions earlier so that everyone can build on the same understanding.
Identifying this minimum capacity also established the engine configuration. Under the assignment rules, aircraft with at least 300 seats must use four engines. Therefore, every feasible design required four engines, which determined the fuel consumption rate and fixed cost used in our comparisons. Editing the Doeing 7700 introduction helped me practise explaining these connected decisions clearly.

Through this assignment, I learned to check constraints before comparing costs. I initially focused on individual aircraft characteristics, but I came to understand that capacity, flight frequency and operating costs must be considered together. I also found that a small calculation could provide useful direction for the whole team. The project strengthened my confidence in explaining numerical reasoning and showed me the value of sharing assumptions early. In future group work, I would document these checks at the beginning so that everyone can build on the same understanding.

</details>

<details>
<summary><strong>Member 3</strong></summary>

<!-- 把 Member 3 换成你的姓名。在这里粘贴 200–250 词英文反思，写清实际分工、遇到的问题、怎么处理和学到了什么。正文前后留空行。 -->

</details>

<details>
<summary><strong>Member 4</strong></summary>

<!-- 把 Member 4 换成你的姓名。在这里粘贴 200–250 词英文反思，写清实际分工、遇到的问题、怎么处理和学到了什么。正文前后留空行。 -->

</details>

<details>
<summary><strong>Member 5</strong></summary>

<!-- 把 Member 5 换成你的姓名。在这里粘贴 200–250 词英文反思，写清实际分工、遇到的问题、怎么处理和学到了什么。正文前后留空行。 -->

</details>

<details>
<summary><strong>Member 6</strong></summary>

<!-- 把 Member 6 换成你的姓名。在这里粘贴 200–250 词英文反思，写清实际分工、遇到的问题、怎么处理和学到了什么。正文前后留空行。 -->

</details>

<details>
<summary><strong>Member 7</strong></summary>

<!-- 把 Member 7 换成你的姓名。在这里粘贴 200–250 词英文反思，写清实际分工、遇到的问题、怎么处理和学到了什么。正文前后留空行。 -->

</details>

<details>
<summary><strong>Member 8</strong></summary>

<!-- 把 Member 8 换成你的姓名。在这里粘贴 200–250 词英文反思，写清实际分工、遇到的问题、怎么处理和学到了什么。正文前后留空行。 -->

</details>

