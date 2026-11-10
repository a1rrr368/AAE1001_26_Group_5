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

The current `Task 1_Final` returns a weighted route time of **58.7875 minutes**. A330-900neo has the lowest total cost in all three scenarios. A321neo requires 17 flights in Scenario 1 and 8 flights in Scenario 2, exceeding the respective limits; all three aircraft are feasible in Scenario 3.

The calculation uses `N = ceil(passengers / capacity)` and `total_cost = N * (fuel_price * fuel_rate * T_best + C_T * T_best + C_c)`. Totals use the unrounded route time and are rounded only for display.

**Current implementation assumptions:** cardinal moves cost 1 and diagonal moves cost `sqrt(2)`. Entering the fuel/time regions adds 15%/30% to the same weighted route cost, which is then used as `T_best` for both fuel and time charges. These are results of the current code. The lecture specifies 2 minutes for diagonal moves; this difference, and the treatment of fuel penalties as weighted time, should be resolved before claiming full compliance. The route image above is retained as the original reference image and has not been regenerated for this update.

## Task 2

### Jet Stream Placement

The model adds a horizontal jet stream that spans the map and is five units high. The current script sets its lower boundary to y = 5 and reduces the base movement cost in the band by 5%. The comparison uses Scenario 1.

<!-- 下面已插入仓库现有的 Task 2 图。如果 PPT 用了另一张图，把原图放进仓库并改路径。若直接粘贴图片失败，可先存为 PNG，再用 ![图片英文说明](./文件名.png) 引用。 -->

![Task 2 route with the jet stream](./Task%202%20path.png)

<!-- 这里填无气流、y = 5 和其他实际测试位置。填好后用一两句说明哪个测试位置成本最低，以及比无气流省多少。 -->

<!-- UPDATE T2: baseline comparison using Task 1_Final and Task 2_Final. | README line 74. -->

| Jet stream placement | Route time (min) | Scenario 1 total cost ($) |
|---|---:|---:|
| No jet stream | 58.7875 | 81,751.58 |
| Lower boundary y = 5 | 58.5960 | 81,557.02 |

Both rows select **11 A330-900neo flights**. The tested jet stream saves approximately **$194.56 (0.238%)**, using `(baseline - jet_stream_cost) / baseline * 100`. The 5% reduction applies to base movement cost inside the band, rather than to the entire trip or fixed costs. Regional surcharges remain additive in the current implementation.

Only the current lower boundary `y = 5` has been verified here; it is not established as the best placement over all possible positions. The current A* heuristic remains unscaled despite discounted edges, so global optimality of the discounted search has not been established. The existing image is retained as a reference and has not been regenerated.

## Task 3

### New Aircraft Design

The design must carry 3,300 passengers in at most 13 flights. Candidate capacities range from 100 to 450 seats in steps of 50. Capacities below 300 seats use two engines, and capacities from 300 seats onward use four. The model compares fuel, time, and fixed costs.

<!-- 用 Task 3_Final 核对各座位档位。填每班三类费用与总成本，在表下写最终机名、座位数、发动机数和班次，并说明为什么选它。 -->

<!-- UPDATE T3: cost breakdown verified against Task 3_Final; comparison with Task 1 baseline. | README line 93. -->

| Seats | Flights required | Engines | Fuel per flight ($) | Time per flight ($) | Fixed per flight ($) | Total cost ($) |
|---:|---:|---:|---:|---:|---:|---:|
| 100–250 | Over 13 | 2 | — | — | — | Infeasible |
| 300 | 11 | 4 | 3,997.55 | 1,175.75 | 2,500.00 | 84,406.30 |
| 350 | 10 | 4 | 3,997.55 | 1,293.33 | 2,500.00 | 77,908.75 |
| 400 | 9 | 4 | 3,997.55 | 1,410.90 | 2,500.00 | 71,176.05 |
| **450** | **8** | **4** | **3,997.55** | **1,528.48** | **2,500.00** | **64,208.20** |

The selected design is **doeing 7700: 450 seats, four engines and eight flights**. It provides 3,600 seats for 3,300 passengers and stays within the 13-flight limit. Capacities of 100, 150, 200 and 250 require 33, 22, 17 and 14 flights respectively and are infeasible.

The script uses `C_T = 12 + 2 * ((seats - 100) // 50)`, so 100 seats correspond to $12/min. It fixes `T_best = 58.7875` minutes rather than importing Task 1 output. Four engines consume 80 kg/min, giving 4,703 kg of fuel per flight at $0.85/kg. Costs are calculated before rounding the displayed components.

Compared with the 300-seat candidate, the selected design saves **$20,198.10 (23.93%)**. Compared with Task 1 Scenario 1's selected A330-900neo, it saves approximately **$17,543.38 (21.46%)** under the current model. These comparisons inherit the Task 1 assumptions described above.

<!-- 如果 PPT 有成本对比图，把 PNG 放进仓库，例如 figures/task3-cost.png，再在这里加一行 ![Aircraft design cost comparison](./figures/task3-cost.png)。 -->

## Task A1

### Supply Checkpoints

The planner generates one checkpoint in each cost region. It compares the two possible visit orders and selects the reachable order with the lower weighted travel time.

<!-- 这里写随机种子、两个补给点坐标、两种顺序的用时及最终顺序。把生成的路线图放进仓库，再在这里插入 ![Route through both checkpoints](./figures/task-a1.png)。路径按实际文件名改。 -->

<!-- UPDATE A1: reproducible checkpoint results and existing output options. | README line 119. -->

With the default `--seed 5`, the fuel checkpoint is **(7, 6)** and the time checkpoint is **(59, 1)**.

| Visit order | Total weighted travel time (min) |
|---|---:|
| Start -> Fuel -> Time -> Base | **148.2168** |
| Start -> Time -> Fuel -> Base | 178.0319 |

The selected order visits Fuel first and Time second, reducing weighted time by **29.8151 minutes** relative to the other order. The script checks that both checkpoints are visited and that the base is not reached earlier. By default it saves `A1_results/A1_route.png` and `A1_results/A1_route.txt`; `--output` changes the output path. The generated figure has not been uploaded in this README update.

## Task A2

### Randomized Environment

The program generates a 40 × 40 fuel-cost region, random obstacles, and endpoints at least 40 units apart. The planner moves in four directions.

<!-- 这里放一张实际生成的地图和路线，例如 ![Random map and planned route](./figures/task-a2.png)。图下写起终点距离、障碍密度、路径代价和随机设置。 -->

<!-- UPDATE A2: code parameters and a seeded verification example. | README line 138. -->

The main program uses map bounds `[-10, 60]`, grid resolution 1, robot radius 1 and `obstacle_density=0.15`. This density controls the number of sampled obstacle candidates, not the exact final occupied-cell fraction. Candidates within 4 units of either endpoint are skipped. The minimum sampled start-goal distance is 40 units. Entering the fuel region adds 50% to movement cost.

The script does not set a random seed. For a reproducible verification example, we set Python's global random seed to 5 immediately before calling the unchanged `main()` function:

| Item | Verified example |
|---|---|
| Start (rounded for display) | (39.4, 50.3) |
| Goal (rounded for display) | (-3.3, 22.9) |
| Sampled start-goal distance | 50.7 units |
| Weighted route cost | 98.00 |

These values describe the seeded example, not every ordinary run. No example figure has been uploaded. The current no-path branch breaks out of the search and still reconstructs a goal-only path; failure handling should be corrected before using it to measure success rates.

## Task A3

### A*, Dijkstra, and RRT*

<!-- 用同一张地图和相同的起终点比较。填写同一成本口径下的结果，并记录 RRT* 的种子和迭代数。有对比图就放在表后，例如 ![Routes found by the three planners](./figures/task-a3.png)。 -->

<!-- UPDATE A3: verified route costs; search-only timing from one headless run. | README line 159. -->

| Algorithm | Route cost | Runtime (s) | Notes |
|---|---:|---:|---|
| A* | 58.7875 | 0.1263 | Eight-direction grid; Euclidean heuristic |
| Dijkstra | 58.7875 | 0.1889 | Same grid and edge costs as A* |
| RRT* | 59.7257 | 5.5532 | Continuous segments; seed 0; 5,000 iterations |

All three use the Group 5 map, start **(5, 40)** and goal **(20, 30)**. Timings above are single-run, animation-disabled measurements of `planning()` only; obstacle-map construction, plotting and aircraft-cost calculations are excluded. They are environment-dependent observations, not repeated benchmark averages.

A* and Dijkstra return the same route cost on the shared grid. The tested RRT* run returns a higher native cost. RRT* uses continuous segments with midpoint-sampled regional penalties, whereas the grid algorithms apply penalties at destination cells, so its native cost is not an exactly identical evaluation metric. A shared path evaluator and multiple RRT* seeds are needed for a controlled quality comparison. Default RRT* settings are step size 3, goal sample rate 0.1, maximum 5,000 iterations and seed 0.

## How to Run

<!-- 这里给组员或老师复现。提交前核对 Python 版本、依赖、文件名和命令。若脚本改名，同时更新这里的链接。 -->

<!-- UPDATE RUN: dependency installation, script behaviour and reproducibility commands. | README line 175. -->

Run the scripts from the repository root with Python 3 and Matplotlib installed. The Task 1, Task 2 and A3 scripts import `MatplotlibDeprecationWarning` directly from Matplotlib; their source headers target Matplotlib 3.10+. No exact package version is pinned in the repository.

```bash
python -m pip install "matplotlib>=3.10"
```

The source files without a `.py` extension can still be run using the quoted filenames below.

| Task | Source file | Command |
|---|---|---|
| 1 | [Task 1_Final](./Task%201_Final) | `python "Task 1_Final"` |
| 2 | [Task 2_Final](./Task%202_Final) | `python "Task 2_Final"` |
| 3 | [Task 3_Final](./Task%203_Final) | `python "Task 3_Final"` |
| A1 | [Task A1_Final](./Task%20A1_Final) | `python "Task A1_Final" --no-show` |
| A2 | [Task A2_Final](./Task%20A2_Final) | `python "Task A2_Final"` |
| A3 | [Dijkstra](./Task%20A3.1_Final) and [RRT*](./Task%20A3.2_Final) | `python "Task A3.1_Final"` and `python "Task A3.2_Final"` |

Task 1, Task 2, A2 and A3 display plots by default. Set their existing `show_animation` variable to `False` when collecting numerical results or timings. Task 3 repeatedly asks for a capacity; use 300, 350, 400 or 450 for feasible designs and press Ctrl+C to stop. Its current code does not handle non-integer input or zero safely.

For Task A1, use the existing command-line options:

```bash
python "Task A1_Final" --seed 5 --no-show --output A1_results/A1_route.png
```

To reproduce the A2 seeded example without changing its source:

```bash
python -c 'import random, runpy; random.seed(5); runpy.run_path("Task A2_Final", run_name="__main__")'
```

For the numerical checks documented here, plotting was disabled; no new route images were generated or uploaded.

## Team Members and Contributions

<!-- 保留了草稿中的八行。填真实姓名和实际分工。学号只按课程要求填写。 -->

<!-- UPDATE TEAM: identities and individual allocations still require team confirmation; no details invented. | README line 214. -->

| Name | Student ID | Contribution |
|---|---|---|
| Louis | 26125637D | I completed Task A1 with ChatGPT's help, created the Task 1 presentation, helped clarify the Task 2 requirements, and revised the cost calculations and flight constraints in a teammate’s Task 3 code. |
| Romeo | 26088558D | Refined the programs for all tasks and tailored the aircraft design approach |
| Sahil | 26127898D | Wrote the code that helped do the calculations for Task 3, helped check the best approaches with the jet stream with teammates in Task 2, and developed presentation slides for Task A3 |
| Justin | 24082728D | Revised the draft code of Task 1 to clear out some errors, completed Task A2 with the help of ChatGPT and made the presentation slides for the task. |
| Member 5 | — | — |
| Member 6 | — | — |
| Member 7 | — | — |
| Member 8 | — | — |

## References

<!-- 补上实际借鉴的 Dijkstra、RRT* 原始 GitHub 文件链接。若图片来自外部，也在这里注明出处。 -->

- [AAE1001 course repository](https://github.com/RNS-Lab-PolyU/PolyU_AAE1001_Github_Project_2026)

<!-- UPDATE SOURCES: direct implementation links; external algorithm provenance still to be supplied by authors. | README line 233. -->

The numerical tables above were checked against these repository implementations:

- [Task 1: A* and scenario costs](./Task%201_Final)
- [Task 2: jet stream](./Task%202_Final)
- [Task 3: aircraft capacity and costs](./Task%203_Final)
- [Task A1: checkpoint orders](./Task%20A1_Final)
- [Task A2: random environment](./Task%20A2_Final)
- [Task A3.1: Dijkstra](./Task%20A3.1_Final)
- [Task A3.2: RRT*](./Task%20A3.2_Final)

These links identify the code used for verification. Any external Dijkstra/RRT* sources actually adapted by the authors, and any external image credits, still need to be added; provenance has not been inferred from algorithm names.

## Individual Reflections

<details>
<summary><strong>Louis</strong></summary>

I developed Task A1 with ChatGPT's help and created the presentation slides for Task 1. I also helped the group interpret the requirements for Task 2, revised a teammate’s implementation for Task 3, and organized the project README.

Creating the Task 1 slides taught me to explain the route-planning method and aircraft selection in a sequence the audience could follow. I applied the same idea to the README by organizing the report by task. Across these contributions, I learned that a result is easier to understand and verify when the assumptions, method, and evidence have clear logic.

For Task 2, my role was to clarify the requirements and help plan the comparison. I helped define what we needed to compare across jet stream placements, including route time and total operating cost for Scenario 1. For Task 3, I built on my teammate’s code by correcting parts of the cost calculation and checking the flight limit, which helped me understand how seating capacity affects both the number of flights and the total cost.

For Task A1, I wrote the planner to generate one checkpoint in each cost region and visit both before reaching the destination. I compared the two possible visit orders using the total weighted travel time. Using a fixed random seed made the example reproducible, which helped me check the route and explain why the planner chose that order.

</details>

<details>
<summary><strong>Romeo</strong></summary>

<!-- UPDATE ROMEO: approved reflection including program improvements. -->

I contributed to Task 3 by checking the minimum feasible passenger capacity. Transporting 3,300 passengers within 13 flights requires approximately 254 seats per flight. Since capacities increase in steps of 50, 300 seats was the smallest feasible option; 250 seats would require 14 flights. This helped the team focus on the 300-, 350-, 400- and 450-seat designs. Under the assignment rules, all these options required four engines, establishing the fuel consumption rate and fixed cost for our comparisons.

I also prepared the PowerPoint slides introducing Doeing 7700, edited its introduction, and supplemented the README to explain our design reasoning. In addition, I helped improve and modify the Task 1, Task 2 and Task 3 programs to make them more efficient and check their correctness. This included reducing repeated calculations and event registration, improving region checks, and clarifying comments. I compared outputs before and after the changes to confirm that the routes and operating costs remained unchanged.

This assignment taught me to check constraints before comparing costs and to consider capacity, flight frequency and operating costs together. Improving the programs also showed me that faster execution must be supported by verification, rather than assumed to produce correct results. Explaining our work through the PowerPoint and README strengthened my confidence in communicating numerical reasoning. I learned that calculations and documentation can both help a team make better decisions. In future projects, I would record assumptions earlier and verify changes systematically before sharing them.

</details>

<details>
<summary><strong>Sahil</strong></summary>

In Task 2, a teammate created the code which implemented the jetstream area into the map, to which me and other teammates helped find what was the ideal position to place the jetstream to output the lowest cost.

For Task 3, our objective was to find the ideal plane load that would adhere to scenario 1's requirements, while also keeping costs to a minimum, we had to choose how much passengers to have in increments of 50, and if we exceeded 300 passengers, we would require 4 engines. All these variables added onto the formula given to us by the task itself ( C = (Cf . dF . Tbest) + (Ct . Tbest . Cc) ). I designed a program that did the calculations from the given formula based on our input of passenger load, allowing us to find the ideal load with ease.

Finally I was tasked with creating the presentation slides for Task A3, which was the comparison of two other search algorithms and the A* planner. I conducted research on the two chosen search algorithms and gave a brief explanation on each, also listing their main strength and disadvantage. At the end I also compared the total cost estimated from all three algorithms, and their respective runtimes. 

Ultimately, this project bridged the gap between theoretical aeronautics and computational problem-solving. By automating cost formulas and analyzing environmental variables like jetstreams, I learned how to translate strict operational constraints into efficient programmatic solutions. Furthermore, comparing the A* planner against other search algorithms sharpened my ability to critically evaluate technical approaches, specifically balancing computational runtime against optimization outcomes. Moving forward, the ability to build automated mathematical models and analytically present complex data will be invaluable for tackling advanced aircraft performance analysis, flight routing, and systems design.

</details>

<details>
<summary><strong>Justin</strong></summary>

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


