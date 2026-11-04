from setuptools import setup
setup(name='acg138_semantic', version='0.1.0', packages=['acg138_semantic'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/acg138_semantic']),
                  ('share/acg138_semantic', ['package.xml'])],
      entry_points={'console_scripts': ['semantic_search=acg138_semantic.node:main', 'follow_lidar=acg138_semantic.follow_node:main']})
