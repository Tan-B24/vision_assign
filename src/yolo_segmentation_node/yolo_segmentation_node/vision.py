import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from ultralytics import YOLO
import pycuda.driver as cuda
import pycuda.autoinit
from cv_bridge import CvBridge
from std_msgs.msg import Header

class YOLOSegmentationNode(Node):
    def __init__(self):
        super().__init__('yolo_segmentation_node')
        self.subscription = self.create_subscription(
            Image,
            '/d455_1_rgb_image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Publisher for the processed image
        self.image_publisher = self.create_publisher(Image, '/yolo_segmentation/output_image', 10)
        
        # Paths to your models
        ground_model_path = "../../../ground_best.pt"
        pallet_model_path = "../../../pallet_best.pt"
        
        # Load YOLO models
        self.ground_model = YOLO(ground_model_path)
        self.pallet_model = YOLO(pallet_model_path)
        
        # Bridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = image.shape
        
        # Run both models on the image
        pallet_results = self.run_yolo_inference(self.pallet_model, image)
        ground_results = self.run_yolo_inference(self.ground_model, image)
        
        # Create an overlay for segmentation masks
        mask_overlay = np.zeros_like(image, dtype=np.uint8)
        
        # Draw pallet segmentation results
        for result in pallet_results:
            mask = result['mask']
            if mask is not None:
                # Resize mask to match the original image dimensions
                mask_resized = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
                mask_resized = (mask_resized > 0).astype(np.uint8)  # Convert to binary mask
                mask_overlay[mask_resized == 1] = (255, 0, 0)  # Blue color for pallet

            x_min, y_min, x_max, y_max = result['xmin'], result['ymin'], result['xmax'], result['ymax']
            confidence = result['confidence']
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
            cv2.putText(image, f'Pallet {confidence:.2f}', (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # Draw ground segmentation results
        for result in ground_results:
            mask = result['mask']
            if mask is not None:
                # Resize mask to match the original image dimensions
                mask_resized = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
                mask_resized = (mask_resized > 0).astype(np.uint8)  # Convert to binary mask
                mask_overlay[mask_resized == 1] = (0, 255, 0)  # Green color for ground

        # Combine original image with the mask overlay
        combined_image = cv2.addWeighted(image, 0.7, mask_overlay, 0.3, 0)
        
        # Convert the combined image back to ROS Image message and publish it
        output_msg = self.bridge.cv2_to_imgmsg(combined_image, encoding='bgr8')
        output_msg.header = Header()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_publisher.publish(output_msg)
        
        self.get_logger().info("Processed and published image.")

    def run_yolo_inference(self, model, image, conf_threshold=0.25):
        # Run inference
        results = model(image)
        # Extract results for bounding boxes and masks with confidence filtering
        filtered_results = []
        for result in results:
            boxes = result.boxes.data.cpu().numpy()  # Extract bounding boxes
            masks = result.masks.data.cpu().numpy() if result.masks else None  # Extract masks if available

            for idx, box in enumerate(boxes):
                x_min, y_min, x_max, y_max, confidence, class_id = box
                if confidence >= conf_threshold:
                    mask = masks[idx] if masks is not None else None
                    filtered_results.append({
                        'xmin': int(x_min),
                        'ymin': int(y_min),
                        'xmax': int(x_max),
                        'ymax': int(y_max),
                        'confidence': confidence,
                        'class_id': int(class_id),
                        'mask': mask
                    })
        return filtered_results

def main(args=None):
    rclpy.init(args=args)
    node = YOLOSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
