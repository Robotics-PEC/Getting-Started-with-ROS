######
Setup
######

***************
Prerequisites
***************

- Docker
- Xming (for Windows users)

***************
Installation
***************

1. **Start Docker:**

   .. code-block:: sh

      docker-compose up -d

2. **Attach to Docker container:**
   
   .. code-block:: sh

      docker-compose exec ros bash

=======================
XServer Setup (Windows)
=======================

To forward video from Docker to the base OS, ensure you have Xming installed and running.