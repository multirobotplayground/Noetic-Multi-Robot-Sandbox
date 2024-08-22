<p align="center">
  <img src="docs/images/sandbox_project_logo.png" alt="drawing" style="width:500px;"/>
</p>

# [Table of Contents](#table-of-contents)

- [Setup](docs/working_environment.md)
- [Usage](docs/usage.md)
- [Robots](docs/robots.md)
- [Maps](docs/maps.md)
- [Nodes](docs/multi-robot-exploration.md)
- [Multi-robot Guidelines](docs/multi-robot-guidelines.md)
- [Documentation](https://multirobotplayground.github.io/ROS-Noetic-Multi-robot-Sandbox/html/index.html)
- [Contributing](docs/contributing.md)

## [ROS-Noetic-Multi-robot-Sandbox](#ros-noetic-multi-robot-sandbox)

This workspace is a sandbox for multi-robot research.

Initially, it includes a fully opperational multi-robot exploration stack for ROS Noetic and Ubuntu 20.04 that allows them to keep intermittent communication in Gazebo 11 simulations. It contains an implementation and integration of the structures from this research: [https://arxiv.org/abs/2309.13494](https://arxiv.org/abs/2309.13494). 

The deployment features components highlighted in red in the diagram, allowing robots to explore and maintain intermittent connectivity through a rendezvous plan. Communication is facilitated through topics with a mock communication model.

<p align="center">
<img src="docs/images/stack_components_high.png" alt="drawing" style="width:1000px;"/>
</p>

## [What to Expect?](#what-to-expect)

### Fully opperational multi-robot exploration with communication constraints to accelerate your research and development.

<p align="center">
<img src="docs/images/example_exploration.png" alt="drawing" style="width:800px;"/>
</p>

### Complex stack integrated with customized nodes for ROS Noetic, with SLAM, map stitching, trajectory optimization, global and local planning, and more, to simulate a realistic multi-robot exploration application with communication constraints, visualization, and configurations for Gazebo 11 and rviz.

<p align="center">
<img src="docs/images/stack.png" alt="drawing" style="width:800px;"/>
</p>

### Intermittent Communication Policy to share information at rendezvous locations spread dynamically while exploration happens.

<p align="center">
<img src="docs/images/spread.png" alt="drawing" style="width:800px;"/>
</p>

## [Disclaimer](#disclaimer)

This workspace is not based on the ```move_base``` and the ```nav``` stack for navigation, because they had many drawbacks when I first tried to used them for multi-robot exploration research. Therefore, this project contains its own ```sub-goal``` navigation node with ```global``` and ```local planners``` to handle situations posed by ```multi-robot exploration``` applications when robots do ```SLAM``` individually, have to deal with traffic, and share maps. Furthermore, I'm currently refactoring all nodes to make them more friendly with ROS 2 standards.


## [Features Roadmap](#features-roadmap)

| Feature | Status |
|-|-|
| Enhance Documentation | <span style="color:green">in-progress</span> |
| [ROS 2 Jazzy Jalisco Integration](https://github.com/Ophien/ROS-Jazzy-Multi-robot-Sandbox) | <span style="color:green">in-progress</span> |
| [Add Real World Deployment](https://youtu.be/8FP3UK7kpRA?feature=shared) | |
| Youtube Channel | |
| Add Robust Testing | |
| Add Probabilistic-based Global Planner | |
| Add Pose Graph-base Mapping | |
| Add Pose Graph Merging | |
| Add Heterogeneous Robot Teams Support | |
| Add Human Computer Interfaces for Ground Teams | |
| Add Mission Assignment and Inspector | |
| Add Other Baselines and Algorithms | |
| Add Other Coordination Methods | |
| [Add More Realistic Maps and Environments for Fuel](https://app.gazebosim.org/fuel/models) | |
| [Add Safety Mechanisms for Navigation and Traffic Avoidance](https://youtu.be/HSFmGdI0TAY?si=gvVgttymSCMpddl7) | |
| Unity3D integration | |

## [Publications](#publications)

If this workspace is somehow useful to you, consider reading this [letter](docs/motivation.md) and mentioning this paper accepted on IROS.

> A. R. da Silva, L. Chaimowicz, T. C. Silva, and A. Hsieh, Communication-Constrained Multi-Robot Exploration with Intermittent Rendezvous. 2024.

```text
@misc{dasilva2024communicationconstrained,
      title={Communication-Constrained Multi-Robot Exploration with Intermittent Rendezvous}, 
      author={Alysson Ribeiro da Silva and Luiz Chaimowicz and Thales Costa Silva and Ani Hsieh},
      year={2024},
      eprint={2309.13494},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## [Partnerships](#partnerships)

<p align="center">
<img src="docs/images/penn.png" alt="drawing" style="width:200px;"/>
<img src="docs/images/scalar_logo.png" alt="drawing" style="width:200px;"/>
<br>
<img src="docs/images/ufmg-logo.png" alt="drawing" style="width:200px;"/>
<img src="docs/images/dcc.png" alt="drawing" style="width:200px;"/>
<img src="docs/images/verlab_logo.png" alt="drawing" style="width:200px;"/>
</p>

## [Acknowledgements](#acknowledge)

The research paper [https://arxiv.org/abs/2309.13494](https://arxiv.org/abs/2309.13494) was supported by ARL DCIST CRA W911NF-17-2-0181 and Office of Naval Research (ONR) Award No. N00014-19-1-2253.

<p align="center">
<img src="docs/images/dcist.png" alt="drawing" style="width:200px;"/>
<img src="docs/images/onr.jpg" alt="drawing" style="width:200px;"/>
</p>

It was also supported by CAPES, FAPEMIG, and CNPQ.

<p align="center">
<img src="docs/images/capes.png" alt="drawing" style="width:100px;"/>
<img src="docs/images/fapemig.jpg" alt="drawing" style="width:120px;"/>
<img src="docs/images/cnpq.png" alt="drawing" style="width:200px;"/>
</p>

## [Support this Project](#support-this-project)

Support Open Source mobile robots projects for search and rescue in natural disasters, which is my main motivation. Your donation will make a huge difference!

[![Donate](docs/images/Donate-PayPal-green-usd.png)](https://www.paypal.com/donate/?business=YWAAG5LVWXBQC&no_recurring=0&item_name=Support+Open+Source+mobile+robots+projects+for+search+and+rescue+in+natural+disasters.+Your+donation+can+change+lives%21&currency_code=USD)
[![Donate](docs/images/Donate-PayPal-green-brl.png)](https://www.paypal.com/donate/?business=YWAAG5LVWXBQC&no_recurring=0&item_name=Support+Open+Source+mobile+robots+projects+for+search+and+rescue+in+natural+disasters.+Your+donation+can+change+lives%21&currency_code=BRL)

## [License](#license)

All content from this repository is released under a modified [BSD 4-clause license](LICENSE).

Author/Maintainer:

- [Alysson Ribeiro da Silva](https://alysson.thegeneralsolution.com/)

emails:

- <alysson.ribeiro.silva@gmail.com>


## [Bug & Feature Requests](#bug--feature-requests)

Please report bugs and do your requests to add new features through the [Issue Tracker](https://github.com/multirobotplayground/ROS-Noetic-Multi-robot-Sandbox/issues).
