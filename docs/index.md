# Waiter-Bot documentation

Waiter-Bot is a café butler robot: it takes orders for tables, collects the food from the kitchen, delivers it, and returns home, handling missing confirmations and cancellations on the way. It is the answer to the Goat Robotics ROS developer assessment, rebuilt for **ROS 2 Jazzy and Gazebo Harmonic**.

| Document | What it covers |
|---|---|
| [architecture.md](architecture.md) | Environment findings, design decisions, packages, data flow, and the task state machine |
| [requirements.md](requirements.md) | The assessment's seven situations, pinned down as testable requirements (R1 to R7) |
| [interfaces.md](interfaces.md) | Actions, services, messages, topics, and parameters, with defaults |
| [world_and_robot.md](world_and_robot.md) | Robot model, café layout, location table, frames |
| [test_plan.md](test_plan.md) | Every test by ID; milestones use these as gates |
| [setup.md](setup.md) | What to install and how to build |
| [migration.md](migration.md) | What happened to each file of the first (ROS 1 style) submission |
| [plan/overview.md](plan/overview.md) | Master plan: milestones M0 to M9, order, traceability, risks |

Status: the plan is written; no milestone is implemented yet. Each milestone file in [plan/](plan/) gets a **Result** section when it lands. Together these are the "documented approach for each milestone" the assessment asks for.
