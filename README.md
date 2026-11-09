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

| Scenario | Passengers | Flight limit | Fuel price ($/kg) | Selected aircraft | Flights | Total cost ($) |
|---|---:|---:|---:|---|---:|---:|
| 1 | 3,300 | 13 | 0.85 | — | — | — |
| 2 | 1,500 | 7 | 0.96 | — | — | — |
| 3 | 2,250 | 25 | 0.78 | — | — | — |

## Task 2

### Jet Stream Placement

The model adds a horizontal jet stream that spans the map and is five units high. The current script sets its lower boundary to y = 5 and reduces the base movement cost in the band by 5%. The comparison uses Scenario 1.

<!-- 下面已插入仓库现有的 Task 2 图。如果 PPT 用了另一张图，把原图放进仓库并改路径。若直接粘贴图片失败，可先存为 PNG，再用 ![图片英文说明](./文件名.png) 引用。 -->

![Task 2 route with the jet stream](./Task%202%20path.png)

<!-- 这里填无气流、y = 5 和其他实际测试位置。填好后用一两句说明哪个测试位置成本最低，以及比无气流省多少。 -->

| Jet stream placement | Route time (min) | Scenario 1 total cost ($) |
|---|---:|---:|
| No jet stream | — | — |
| Lower boundary y = 5 | — | — |

## Task 3

### New Aircraft Design

The design must carry 3,300 passengers in at most 13 flights. Candidate capacities range from 100 to 450 seats in steps of 50. Capacities below 300 seats use two engines, and capacities from 300 seats onward use four. The model compares fuel, time, and fixed costs.

<!-- 用 Task 3_Final 核对各座位档位。填每班三类费用与总成本，在表下写最终机名、座位数、发动机数和班次，并说明为什么选它。 -->

| Seats | Flights required | Engines | Fuel per flight ($) | Time per flight ($) | Fixed per flight ($) | Total cost ($) |
|---:|---:|---:|---:|---:|---:|---:|
| 100–250 | Over 13 | 2 | — | — | — | Infeasible |
| 300 | 11 | 4 | — | — | — | — |
| 350 | 10 | 4 | — | — | — | — |
| 400 | 9 | 4 | — | — | — | — |
| 450 | 8 | 4 | — | — | — | — |

<!-- 如果 PPT 有成本对比图，把 PNG 放进仓库，例如 figures/task3-cost.png，再在这里加一行 ![Aircraft design cost comparison](./figures/task3-cost.png)。 -->

## Task A1

### Supply Checkpoints

The planner generates one checkpoint in each cost region. It compares the two possible visit orders and selects the reachable order with the lower weighted travel time.

<!-- 这里写随机种子、两个补给点坐标、两种顺序的用时及最终顺序。把生成的路线图放进仓库，再在这里插入 ![Route through both checkpoints](./figures/task-a1.png)。路径按实际文件名改。 -->

## Task A2

### Randomized Environment

The program generates a 40 × 40 fuel-cost region, random obstacles, and endpoints at least 40 units apart. The planner moves in four directions.

<!-- 这里放一张实际生成的地图和路线，例如 ![Random map and planned route](./figures/task-a2.png)。图下写起终点距离、障碍密度、路径代价和随机设置。 -->

## Task A3

### A*, Dijkstra, and RRT*

<!-- 用同一张地图和相同的起终点比较。填写同一成本口径下的结果，并记录 RRT* 的种子和迭代数。有对比图就放在表后，例如 ![Routes found by the three planners](./figures/task-a3.png)。 -->

| Algorithm | Route cost | Runtime (s) | Notes |
|---|---:|---:|---|
| A* | — | — | Grid search with a heuristic |
| Dijkstra | — | — | Grid search by accumulated cost |
| RRT* | — | — | Sampling-based search |

## How to Run

<!-- 这里给组员或老师复现。提交前核对 Python 版本、依赖、文件名和命令。若脚本改名，同时更新这里的链接。 -->

Run the scripts from the repository root with Python 3 and Matplotlib installed.

| Task | Source file | Command |
|---|---|---|
| 1 | [Task 1_Final](./Task%201_Final) | `python "Task 1_Final"` |
| 2 | [Task 2_Final](./Task%202_Final) | `python "Task 2_Final"` |
| 3 | [Task 3_Final](./Task%203_Final) | `python "Task 3_Final"` |
| A1 | [Task A1_Final](./Task%20A1_Final) | `python "Task A1_Final" --no-show` |
| A2 | [Task A2_Final](./Task%20A2_Final) | `python "Task A2_Final"` |
| A3 | [Dijkstra](./Task%20A3.1_Final) and [RRT*](./Task%20A3.2_Final) | `python "Task A3.1_Final"` and `python "Task A3.2_Final"` |

## Team Members and Contributions

<!-- 保留了草稿中的八行。填真实姓名和实际分工。学号只按课程要求填写。 -->

| Name | Student ID | Contribution |
|---|---|---|
| Member 1 | — | — |
| Member 2 | — | — |
| Member 3 | — | — |
| Member 4 | — | — |
| Member 5 | — | — |
| Member 6 | — | — |
| Member 7 | — | — |
| Member 8 | — | — |

## References

<!-- 补上实际借鉴的 Dijkstra、RRT* 原始 GitHub 文件链接。若图片来自外部，也在这里注明出处。 -->

- [AAE1001 course repository](https://github.com/RNS-Lab-PolyU/PolyU_AAE1001_Github_Project_2026)

## Individual Reflections

<details>
<summary><strong>Louis</strong></summary>

I developed Task A1 with ChatGPT's help and created the presentation slides for Task 1. I also helped the group interpret the requirements for Task 2, revised a teammate’s implementation for Task 3, and organized the project README.

Creating the Task 1 slides taught me to explain the route-planning method and aircraft selection in a sequence the audience could follow. I applied the same idea to the README by organizing the report by task. Across these contributions, I learned that a result is easier to understand and verify when the assumptions, method, and evidence have clear logic.

For Task 2, my role was to clarify the requirements and help plan the comparison. I helped define what we needed to compare across jet stream placements, including route time and total operating cost for Scenario 1. For Task 3, I built on my teammate’s code by correcting parts of the cost calculation and checking the flight limit, which helped me understand how seating capacity affects both the number of flights and the total cost.

For Task A1, I wrote the planner to generate one checkpoint in each cost region and visit both before reaching the destination. I compared the two possible visit orders using the total weighted travel time. Using a fixed random seed made the example reproducible, which helped me check the route and explain why the planner chose that order.

</details>

<details>
<summary><strong>Member 2</strong></summary>

<!-- 把 Member 2 换成你的姓名。在这里粘贴 200–250 词英文反思，写清实际分工、遇到的问题、怎么处理和学到了什么。正文前后留空行。 -->

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
