# Simultaneous Synchronization and Calibration for Wide-baseline Stereo Event Cameras
## ICRA2024 Paper Submission

This repository hosts the codebase and dataset for our research paper, "Simultaneous Synchronization and Calibration for Wide-baseline Stereo Event Cameras," currently under review for ICRA2024.

## Abstract

<div align="center">
    <div align="center">
        <img src="./fig/head_a.png" width="600">
    </div>
    <div style="color: black; font-size: 16px;">
        Illustration of dual-perspective event capture: Two event-based cameras record events of a moving object from divergent viewpoints.
    </div>
</div>

<div align="center">
    <div align="center">
        <img src="./fig/head_b.png" width="600">
    </div>
    <div style="color: black; font-size: 16px;">
        Temporal misalignment: The event streams from the two cameras are asynchronously offset by a temporal discrepancy $t_d$. The curves feature <span style="color:blue">blue</span> and <span style="color:red">red</span> points, signifying the positive and negative events generated during the object's motion, whereas the scattered points illustrate the background noise.
    </div>
</div>




Event-based cameras offer remarkable advantages such as high temporal resolution and low power consumption but suffer from synchronization issues when deployed in multi-camera settings. Our paper introduces a software-based method to achieve millisecond-level synchronization while simultaneously estimating extrinsic parameters. Our approach eliminates the need for specialized hardware, thus making it particularly suitable for wide-baseline configurations. The robustness and applicability of our method are empirically demonstrated through extensive simulations and real-world experiments.



## Usage


## Results


## Contributions
- Novel software-based approach for temporal synchronization of event-based cameras in wide-baseline settings.
- Simultaneous estimation of extrinsic parameters, thus integrating these processes for increased efficiency.
- Comprehensive validation of the method through simulations and real-world experiments.

## Contact

For further inquiries or questions, please contact us at [wlxing@connect.hku.hk].

Thank you for your interest in our research.

