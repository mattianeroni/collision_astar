from setuptools import setup

setup(
    name='collison_astar',
    version='0.0.1',
    description='A reimplementation of A star algorithm that consider anti-collision and collisions risks.',
    author='Mattia Neroni, Ph.D, Eng.',
    author_email='mattianeroni@yahoo.it',
    url='https://github.com/mattianeroni/collision-astar',
    packages=[
        "src"
    ],
    python_requires='>=3.9',
    classifiers=[
        "Development Status :: 3 - Alpha"
    ]
)