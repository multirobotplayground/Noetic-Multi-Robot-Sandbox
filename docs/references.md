# Related Works

This is a complementary literature review for the research paper: [https://arxiv.org/abs/2309.13494](https://arxiv.org/abs/2309.13494)

There are several strategies to solve the Multi-robot Exploration and share information with other agents. For example, one might let the robots explore and meet opportunistically [[1](https://www.sciencedirect.com/science/article/abs/pii/S0921889099000469),[2](https://dl.acm.org/doi/10.5555/647288.723404),[3](https://ieeexplore.ieee.org/document/844100),[4](https://ieeexplore.ieee.org/document/1013690),[5](https://ieeexplore.ieee.org/document/10341485)]. Differently, a base station can be used to ensure that the information is gathered and shared at a specific rate [[6](https://ieeexplore.ieee.org/document/5980179), [7](https://link.springer.com/chapter/10.1007/978-3-662-43645-5_5), [8](https://ieeexplore.ieee.org/document/9361138)]. Another approach would be to let the robots remain connected and move as a structure that can adapt to accomplish tasks [[9](https://ieeexplore.ieee.org/document/7889035), [10](https://ieeexplore.ieee.org/document/8610112), [11](https://ieeexplore.ieee.org/document/9197109), [12](https://ieeexplore.ieee.org/document/9683512)]. Additionally, a relay network can be used, where some robots act as data mules or static network relays [[13](https://ieeexplore.ieee.org/document/9837416)]. As an alternative, robots can plan their trajectories to be intermittently connected [[14](https://ieeexplore.ieee.org/document/6177277), [15](https://ieeexplore.ieee.org/document/8612974), [16](https://ieeexplore.ieee.org/document/9341636)]. These strategies have their advantages in meeting some requirements. However, they can also introduce constraints into the robots’ motion, which is what we try to avoid.

# References

[[1] B. Yamauchi, “Decentralized coordination for multirobot exploration,” Robotics and Autonomous Systems, vol. 29, no. 2, pp. 111–118, 1999.[Online]. Available: https://www.sciencedirect.com/science/article/pii/S0921889099000469](https://www.sciencedirect.com/science/article/abs/pii/S0921889099000469)

[[2] R. Simmons, D. Apfelbaum, W. Burgard, M. Fox, D. an Moors, S. Thrun, and H. Younes, “Coordination for multi-robot exploration and mapping,” in Proceedings of the AAAI National Conference on Artificial Intelligence. Austin, TX: AAAI, 2000.](https://dl.acm.org/doi/10.5555/647288.723404)

[[3] W. Burgard, M. Moors, D. Fox, R. Simmons, and S. Thrun, “Collaborative multi-robot exploration,” in Proceedings 2000 ICRA. Millennium Conference. IEEE International Conference on Robotics and Automation. Symposia Proceedings (Cat. No.00CH37065), vol. 1, 2000, pp. 476–481 vol.1.](https://ieeexplore.ieee.org/document/844100)

[[4] R. Zlot, A. Stentz, M. Dias, and S. Thayer, “Multi-robot exploration controlled by a market economy,” in Proceedings 2002 IEEE International Conference on Robotics and Automation (Cat. No.02CH37292), vol. 3, 2002, pp. 3016–3023 vol.3.](https://ieeexplore.ieee.org/document/1013690)

[[5] S. Bone, L. Bartolomei, F. Kennel-Maushart, and M. Chli, “Decentralised multi-robot exploration using monte carlo tree search,” in 2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2023, pp. 7354–7361.](https://ieeexplore.ieee.org/document/10341485)

[[6] E. Stump, N. Michael, V. Kumar, and V. Isler, “Visibility-based deployment of robot formations for communication maintenance,” in 2011 IEEE International Conference on Robotics and Automation, 2011, pp. 4498–4505.](https://ieeexplore.ieee.org/document/5980179)

[[7] V. Spirin, S. Cameron, and J. de Hoog, “Time preference for information in multi-agent exploration with limited communication,” in Towards Autonomous Robotic Systems, A. Natraj, S. Cameron, C. Melhuish, and M. Witkowski, Eds. Berlin, Heidelberg: Springer Berlin Heidelberg, 2014, pp. 34–45.](https://link.springer.com/chapter/10.1007/978-3-662-43645-5_5)

[[8] L. Clark, J. Galante, B. Krishnamachari, and K. Psounis, “A queue stabilizing framework for networked multi-robot exploration,” IEEE Robotics and Automation Letters, vol. 6, no. 2, pp. 2091–2098, 2021.](https://ieeexplore.ieee.org/document/9361138)

[[9] A. Gasparri, L. Sabattini, and G. Ulivi, “Bounded control law for global connectivity maintenance in cooperative multirobot systems,” IEEE Transactions on Robotics, vol. 33, no. 3, pp. 700–717, 2017.](https://ieeexplore.ieee.org/document/7889035)

[[10] K. Khateri, M. Pourgholi, M. Montazeri, and L. Sabattini, “A comparison between decentralized local and global methods for connectivity maintenance of multi-robot networks,” IEEE Robotics and Automation Letters, vol. 4, no. 2, pp. 633–640, 2019.](https://ieeexplore.ieee.org/document/8610112)

[[11] B. Capelli and L. Sabattini, “Connectivity maintenance: Global and optimized approach through control barrier functions,” in 2020 IEEE International Conference on Robotics and Automation (ICRA), 2020, pp. 5590–5596.](https://ieeexplore.ieee.org/document/9197109)

[[12] P. Ong, B. Capelli, L. Sabattini, and J. Cort´es, “Network connectivity maintenance via nonsmooth control barrier functions,” in 2021 60th IEEE CDC, 2021, pp. 4786–4791.](https://ieeexplore.ieee.org/document/9683512)

[[13] M. Saboia, L. Clark, V. Thangavelu, J. A. Edlund, K. Otsu, G. J. Correa, V. S. Varadharajan, A. Santamaria-Navarro, T. Touma, A. Bouman, H. Melikyan, T. Pailevanian, S.-K. Kim, A. Archanian, T. S. Vaquero, G. Beltrame, N. Napp, G. Pessin, and A.-a. Agha-mohammadi, “Achord: Communication-aware multi-robot coordination with intermittent connectivity,” IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 10 184–10 191, 2022.](https://ieeexplore.ieee.org/document/9837416)

[[14] G. A. Hollinger and S. Singh, “Multirobot coordination with periodic connectivity: Theory and experiments,” IEEE Transactions on Robotics, vol. 28, no. 4, pp. 967–973, 2012.](https://ieeexplore.ieee.org/document/6177277)

[[15] Y. Kantaros, M. Guo, and M. M. Zavlanos, “Temporal logic task planning and intermittent connectivity control of mobile robot networks,” IEEE Trans. on Automatic Control, vol. 64, no. 10, pp. 4105–4120, 2019.](https://ieeexplore.ieee.org/document/8612974)

[[16] H. Rovina, T. Salam, Y. Kantaros, and M. Ani Hsieh, “Asynchronous adaptive sampling and reduced-order modeling of dynamic processes by robot teams via intermittently connected networks,” in 2020 IEEE/RSJ IROS, 2020, pp. 4798–4805.](https://ieeexplore.ieee.org/document/9341636)