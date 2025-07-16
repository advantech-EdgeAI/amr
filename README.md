# Advantech AMR

AMR(autonomous mobile robot) are self-guided machines that can navigate and move in dynamic environments without human intervention or fixed infrastructure like wires or tape. AMR utilize sensors, AI, and machine learning to make real-time decisions for navigation and task execution.

<a href="https://youtu.be/HAqT9Urdels"><img src="https://github.com/user-attachments/assets/e3d337df-a009-4d41-906b-936fb5eea78f"></a>

Advantech ISP team release a software package combined with MIC-732, allow SI quickly deploy AMR or learn ReMEmbR architecture.

# MIC-732
## Features
AI Inference System Based on NVIDIA Nova Orin for AMR Applications

- Fanless and ultra-compact design
- Embedded with NVIDIA® Jetson AGX Orin™ up to 275 TOPS
- Supports 1 x 10GbE, 1 x 2.5GbE, 3 x USB 3.2 Gen 2 (10 Gbit/s)
- Supports 2 x CANbus, 1 x mPCIe, 2 x Nano SIM slots
- Support Total 8-ch GMSL3.0/2.0 with FAKRA connectors
- Support NVIDIA Isaac Robot Operating System (ROS2)

![image](https://github.com/user-attachments/assets/b9e4b16a-96fc-4006-aecd-8d522a5e556a)


# ReMEmbR

ReMEmbR is a workflow that combines LLMs, VLMs and RAG (Retrieval-Augmented Generation) to enable robots to reason, answer questions, and take navigation actions in large areas using long-time memory action. By bringing the power of LLMs and VLMs to tackle reasoning, ReMEmbR strengthens the reasoning and adaptability of an integrated AI-based robotics stack.

With sensor collaboration, Geomatics data being created real-time and save into database. After that, user can interact with robot by microphones.

![image](https://github.com/user-attachments/assets/e39e73d7-e8f4-4aa3-acd7-42e1bf3ccab1)

Check out the demo [Video](https://youtu.be/IyAnNpTZ8q8) and see how we setup an AMR assistant in office. 

## Memory phase

![Memory_Phase](https://github.com/user-attachments/assets/92008880-2cea-47bc-b6d1-00cbb1d2851b)

Memory-building phase is all about making memory work for robots. We take short segments of video, caption information with the captioner node, and then embed information including timestamps and coordinate into a vector database.

![image](https://github.com/user-attachments/assets/4424c6eb-672a-41aa-81af-e5ea2529e268)

## Query phase

![Query_Phase](https://github.com/user-attachments/assets/07966a6b-8b50-479d-b917-3cafcbdad8ab)

When user poses a question to robot, the LLM generates queries to the database, retrieving relevant information iteratively. The LLM can query for text information, position information depending on what the user is asking. This process repeats until the question is answered. 

![image](https://github.com/user-attachments/assets/86620ff0-c322-46cb-9960-3ba447df2408)

# Installation Guide
## System Requirement

## MIC-732 installation

## Server Installation

# Reference
- [MIC-732-AO](https://www.advantech.com/zh-tw/products/965e4edb-fb98-429e-89ed-9a0a8435a7be/mic-732-ao/mod_232b907c-a285-452f-ac0c-28fdadd7d041)
- [Advantech Enables Service AMR with ADATA and NVIDIA ReMEmbR](https://www.advantech.com/en-us/resources/news/advantech-at-gtc-2025-showcasing-cutting-edge-edge-ai-systems-software-innovation-and-ecosystem-partnerships-for-industrial--healthcare-ai#1)
- [Advancing Robot Mobility and Whole-Body Control with Novel Workflows and AI Foundation Models from NVIDIA Research](https://developer.nvidia.com/blog/r2d2-advancing-robot-mobility-whole-body-control-with-ai-from-nvidia-research/)
- [Using Generative AI to Enable Robots to Reason and Act with ReMEmbR](https://developer.nvidia.com/blog/using-generative-ai-to-enable-robots-to-reason-and-act-with-remembr/?linkId=100000291727268&fbclid=IwY2xjawFgnlBleHRuA2FlbQIxMAABHbmgOaT8yNc6oe38kX_gvOSB85J_8tJbao-w1p4rtLY2GTj11lf36M4qIg_aem_YTzEU9oiVql78mXb1lhbWg&ncid=so-face-741088)

Looking for tech support or have a business inquiry? Let’s talk: [Contact Form](https://www.advantech.com/en/form/2bcb7004-44e9-4e70-9ef0-520f326e6141?callback=f51f1493-33ae-43e5-8172-cb8055499ec1)