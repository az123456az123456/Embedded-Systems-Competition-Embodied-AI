from setuptools import setup
from glob import glob
setup(name='acg138_semantic_navigation', version='0.1.0',
      packages=['acg138_semantic_navigation'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/acg138_semantic_navigation']),
                  ('share/acg138_semantic_navigation', ['package.xml']),
                  ('share/acg138_semantic_navigation/launch', glob('launch/*.launch.py'))],
      entry_points={'console_scripts': ['semantic_node = acg138_semantic_navigation.node:main']})
