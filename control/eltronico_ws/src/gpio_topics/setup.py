from setuptools import setup
package_name = 'gpio_topics'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/hook', ['hooks/ament_prefix_path.dsv', 'hooks/ament_prefix_path.sh'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='eltronico',
      maintainer_email='eltronico@localhost', description='ROS 2 GPIO topics using libgpiod',
      license='Apache-2.0', entry_points={'console_scripts': ['gpio_topics_node = gpio_topics.node:main']})
