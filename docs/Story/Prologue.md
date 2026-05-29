# Hatch (孵): A Derived Architecture

## Prologue: Why Build Another Robot Platform?

I needed to weld. Electric arc welding — a robot arm holding a torch, tracing seams on steel plates. I started with ROS and C++, as everyone did. It worked, eventually. But the code was hard to write, harder to debug, and hardest to change. Every small modification rippled through launch files, message definitions, and build configurations. I was spending more time fighting the platform than solving the welding problem.

Then I found Python. The same logic that took hundreds of lines in C++ took dozens in Python. The robot moved. The torch traced. But ROS was still underneath — a distributed system designed for warehouses full of robots and computers, when all I had was one robot and one laptop. I was carrying a backpack full of tools I never used, and they were heavy.

So I asked: what do I actually need?

I need to describe a robot. I need to see it on screen. I need to move its joints, position its tool, and watch it respond. I need to connect to real hardware when it's time to weld, and simulate when I'm testing. That's six needs. Not sixty. Not six hundred.

Every line of code in Hatch exists because one of those needs demanded it. Nothing was added because another platform does it. Nothing was kept because it's conventional. This document traces each need to the component it created, and each component to the principle it revealed.

Hatch is not a collection of tools. It is a chain of reasoning. This is that chain.
