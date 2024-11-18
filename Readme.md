# Vision Assignment

This ROS2 package implements the vision assignment, involving object detection and segmentation using a YOLOv11 model. The entire workflow, from annotation to prediction, is described below in detail.

## Overview

1. #### Annotation with GroundingDINO and Manual Tools
    Images were annotated using GroundingDINO and a manual tool, generating bounding boxes for two classes: `pallet` and `ground`. These bounding boxes were saved in a JSON file for further processing.

2. #### Bounding Box Extraction and Masking with SAM2
    The bounding boxes from the JSON file were input to SAM2 to create segmentation masks. These masks provided accurate pixel-level information for both classes.

3. #### Conversion to YOLO Label Format
    The segmentation masks were converted to a label format compatible with YOLOv11-seg, and saved as text files. These labels defined the object boundaries and classes required for training.

4. #### Preparation of Training Dataset
    A custom `.yaml` file was created, referencing the dataset folders: `/train`, `/val`, and `/test`. Each folder contained images and corresponding labels, structuring the data for YOLO training.

5. #### Model Training and Iterative Improvement
    The YOLOv11-seg model was trained iteratively, using the dataset to detect and segment `pallet` and `ground` classes. Hyperparameters and data augmentation were adjusted to improve model performance, resulting in .pt files representing trained model weights.

5. #### Testing and Model Selection
    The trained `.pt` models were evaluated on the test dataset, and the best-performing model was selected for further predictions. This selection ensured accurate detection and segmentation for the vision task.

## Environment Setup
1. Make sure you have the `ultralytics` library installed before performing the experiment.
```bash
pip install ultralytics
```

## Steps to reproduce the results

1. Clone the repository:

```bash
git clone https://github.com/Tan-B24/vision_assign.git
```

2. Building the workspace using ROS2:

```bash
cd ~/vision_assign
colcon build --symlink-install
```

3. Modify `vision.py` 
    
    1. Navigate to `/src/yolo_segmentation_node/yolo_segmentation_node/` 
    
    2. Update the subscribed camera topic from `'/d455_1_rgb_image'` to match the topic published by your camera.


4. Run the code and visualize the results:

    1. Source the workspace:
        ```bash
        source install/setup.bash
        source /opt/ros/humble/setup.bash
        ros2 run yolo_segmentaion_node yolo_segmentation_node
        ```

    2. To visualize the output:
        
        1. open another terminal and add the image topic under `/yolo_segmentation/output_image`
        ```bash
        rviz2
        ```

## Results

The following video shows the implemented models working in an environment downloaded from `https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/resources/r2bdataset2023`, ROS2 bag file used: r2b_storage

[video](<../Videos/Screencasts/Screencast from 11-17-2024 08:13:30 PM.webm>)