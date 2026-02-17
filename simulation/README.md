# Simulation Asset Utilities

https://josephandrews.notion.site/Simulation-Overview-Goals-2f06b3c9bc768077a774eab623eab0bb?source=copy_link

Two scripts are implemented to support the URDF → USD asset pipeline for Isaac Lab:

* **`clean_urdf.py`**
  Sanitises URDF `xyz` and `rpy` attributes to remove floating-point noise prior to import.

  https://josephandrews.notion.site/URDF-Pre-Processing-Utility-clean_urdf-py-3086b3c9bc7680cabbbfd416c5c94e0e?source=copy_link

* **`convert_urdf_to_usd.py`**
  Converts the humanoid URDF model into a simulation-ready USD articulation using Isaac Lab.

  https://josephandrews.notion.site/URDF-to-USD-Conversion-Pipeline-3086b3c9bc7680a097f4d6756f6aeee9?source=copy_link



---
