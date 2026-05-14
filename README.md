<div align="center">

# Course Assignments — M.Sc. Autonomous Systems, HBRS

[![Jupyter](https://img.shields.io/badge/Jupyter-Notebook-F37626?style=flat-square&logo=jupyter&logoColor=white)](#)
[![Python](https://img.shields.io/badge/Python-3.8+-3776AB?style=flat-square&logo=python&logoColor=white)](#)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=flat-square&logo=ros&logoColor=white)](#)

**Assignments and project notebooks from the Master's programme in Autonomous Systems at Hochschule Bonn-Rhein-Sieg (HBRS).**

</div>

---

## Table of Contents

- [Overview](#overview)
- [Courses](#courses)
  - [Artificial Intelligence](#1-artificial-intelligence)
  - [Autonomous Mobile Robots](#2-autonomous-mobile-robots)
  - [Natural Language Processing](#3-natural-language-processing)
  - [Software Engineering for Robotics](#4-software-engineering-for-robotics)
- [Technology Stack](#technology-stack)
- [Repository Structure](#repository-structure)

---

## Overview

This repository contains coursework from four core courses in the M.Sc. Autonomous Systems programme. Each course folder includes Jupyter notebooks with implementations, PDF reports with theoretical analysis, and supporting data files.

---

## Courses

### 1. Artificial Intelligence

**Path:** `Artificial Intelligence/`

Covers classical AI search algorithms, optimization, and constraint satisfaction.

| Assignment | Topic | Format |
|-----------|-------|--------|
| `Complexity_and_Uninformed_Search.ipynb` | BFS, DFS, complexity analysis, maze solving | Notebook |
| `Informed_Search.ipynb` | A* search, heuristic design, greedy best-first | Notebook |
| `Local_Search.ipynb` | Hill climbing, simulated annealing, genetic algorithms | Notebook |
| `Constraint_Satisfaction_Problems.ipynb` | CSP formulation, backtracking, arc consistency | Notebook |
| `AI_HW01_AhsanNuhel_LintaJoy.pdf` | Homework 1 — Theoretical analysis | Report |
| `AI_HW02_AhsanNuhel_LintaJoy.pdf` | Homework 2 — Theoretical analysis | Report |

**Key topics:** State-space search · Heuristic functions · Optimization · Constraint propagation

---

### 2. Autonomous Mobile Robots

**Path:** `Autonomous Mobile Robots/`

Covers robot kinematics, sensor processing, motion planning, and SLAM fundamentals.

| Assignment | Topic | Format |
|-----------|-------|--------|
| `AMR_WS24_Assignment01.ipynb` | Robot kinematics and motion models | Notebook |
| `AMR_WS24_Assignment05.ipynb` | Sensor processing and localization | Notebook |
| `AMR_WS24_Assignment06.ipynb` | Path planning and navigation | Notebook |

**Key topics:** Kinematics · Sensor fusion · Localization · Path planning · SLAM

---

### 3. Natural Language Processing

**Path:** `NLP/`

Covers fundamental NLP techniques from tokenization to word embeddings.

| Assignment | Topic | Format |
|-----------|-------|--------|
| `BytePairEncoding.ipynb` | BPE tokenization algorithm implementation | Notebook |
| `N-Grams.ipynb` | N-gram language models, perplexity evaluation | Notebook |
| `SimpleEmbeddings.ipynb` | Word embedding techniques (Word2Vec, GloVe concepts) | Notebook |
| `VectorSimilarity.ipynb` | Cosine similarity, distance metrics for text | Notebook |

**Key topics:** Tokenization · Language modeling · Word embeddings · Similarity metrics

---

### 4. Software Engineering for Robotics

**Path:** `Software Engineering for Robotics/`

Covers software design patterns, testing practices, and system architecture for robotic systems.

| Assignment | Topic | Format |
|-----------|-------|--------|
| `state-machines-and-behaviour-trees.ipynb` | FSM and behavior tree design for robot control | Notebook |
| `multi_robot_system/` | Multi-robot coordination implementation | Project |
| `Simulators/` | Simulation environment configurations | Project |
| `testing-and-quality-assurance-ast_ninjas-Simulators/` | Testing and QA for robotic simulators | Project |

**Key topics:** State machines · Behavior trees · Multi-robot systems · Testing · QA

---

## Technology Stack

| Category | Tools |
|----------|-------|
| Programming | Python, Jupyter Notebook |
| Numerical | NumPy, SciPy, Matplotlib |
| ML/AI | scikit-learn |
| Robotics | ROS2, Gazebo |
| NLP | NLTK, custom implementations |

---

## Repository Structure

```
Course-Assignments-HBRS-Masters-Autonomous-Systems/
│
├── Artificial Intelligence/
│   └── Assignments/
│       ├── Complexity_and_Uninformed_Search.ipynb
│       ├── Informed_Search.ipynb
│       ├── Local_Search.ipynb
│       ├── Constraint_Satisfaction_Problems.ipynb
│       ├── AI_HW01_AhsanNuhel_LintaJoy.pdf
│       └── AI_HW02_AhsanNuhel_LintaJoy.pdf
│
├── Autonomous Mobile Robots/
│   └── Assignments/
│       ├── AMR_WS24_Assignment01.ipynb
│       ├── AMR_WS24_Assignment05.ipynb
│       └── AMR_WS24_Assignment06.ipynb
│
├── NLP/
│   └── Assignments/
│       ├── BytePairEncoding.ipynb
│       ├── N-Grams.ipynb
│       ├── SimpleEmbeddings.ipynb
│       └── VectorSimilarity.ipynb
│
└── Software Engineering for Robotics/
    └── Assignments/
        ├── state-machines-and-behaviour-trees.ipynb
        ├── multi_robot_system/
        ├── Simulators/
        └── testing-and-quality-assurance-ast_ninjas-Simulators/
```

---

## Acknowledgments

Hochschule Bonn-Rhein-Sieg (HBRS) · M.Sc. Autonomous Systems Programme · Erasmus Mundus
