from setuptools import find_packages, setup

package_name = "thesis_validation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Luis",
    maintainer_email="luis@todo.todo",
    description="Advance 2 experimental validation and traceability tools.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scenario_recorder = thesis_validation.scenario_recorder:main",
            "validation_summary = thesis_validation.summary:main",
        ],
    },
)
