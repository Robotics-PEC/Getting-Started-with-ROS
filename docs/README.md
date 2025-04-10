# 🧠 Project Documentation

This is the documentation for the **Getting Started with ROS** project, generated using [Sphinx](https://www.sphinx-doc.org/) with the [Read the Docs](https://sphinx-rtd-theme.readthedocs.io/) theme. The site is built and hosted via **GitHub Pages**.

## 📚 Features

- 📘 Built with [Sphinx](https://www.sphinx-doc.org/en/master/)
- 🎨 Styled using the [Read the Docs theme](https://github.com/readthedocs/sphinx_rtd_theme)
- 🚀 Live preview support
- 🌐 Deployed via GitHub Pages

---

## 👨‍💻 Getting Started

You can build and preview the documentation locally on **Linux**, **macOS**, and **Windows**.

### ✅ Prerequisites

- Python 3.10+
- `pip`
- `make` (on Linux/macOS)

---

## 💻 Setup Instructions

### 🔧 Linux & macOS

```bash
make setup       # Create virtual environment & install dependencies
make html        # Build the docs (output in build/html)
make dev         # Start live preview server (auto-reloads on changes)
make clean       # Delete build directory
make fullclean   # Delete build directory and virtual environment
```

### Windows (Command Prompt or PowerShell)

```bat
make.bat setup       :: Create virtual environment & install dependencies
make.bat html        :: Build the docs (output in build/)
make.bat dev         :: Start live preview server (auto-reloads on changes)
make.bat clean       :: Delete build directory
make.bat fullclean   :: Delete build directory and virtual environment
```

---

## 📦 Deployment

This documentation is deployed automatically to **GitHub Pages** from the `gh-pages` branch. Make sure to:

- Build your site with `make html`
- Push the contents of `build/html/` to the `gh-pages` branch (or use GitHub Actions)

---
