#!/usr/bin/env python3

# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

"""
Explore Lite Map Converter Node for Antrobot ROS

This node converts probabilistic occupancy grid maps (like those from Cartographer)
to discrete occupancy grids that work better with frontier-based exploration algorithms.

Two-stage conversion process:
Stage 1 - Cartographer-style thresholding:
- Values below free_threshold (0.1): FREE (0)
- Values above occupied_threshold (0.9): OCCUPIED (100) 
- Unknown values (-1): stay UNKNOWN (-1)
- Values between thresholds: left for Stage 2

Stage 2 - KNN for intermediate values:
- Use adjacent neighbors to decide FREE/OCCUPIED for intermediate probability values
- Creates clean discrete maps suitable for frontier-based exploration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
import math


class ExploreLiteMapConverter(Node):
    """
    Converts probabilistic occupancy grid maps to discrete maps for better frontier detection.
    """

    def __init__(self):
        super().__init__('explore_lite_map_converter')
        
        # Declare parameters with defaults
        self.declare_parameter('enable_explore_lite', False)
        self.declare_parameter('free_threshold', 10)          # Below this: FREE (0.1 probability)
        self.declare_parameter('occupied_threshold', 90)      # Above this: OCCUPIED (0.9 probability)
        self.declare_parameter('use_knn', True)               # Enable KNN for intermediate values
        self.declare_parameter('input_map_topic', 'map')
        self.declare_parameter('output_map_topic', 'map_explore')
        self.declare_parameter('publish_rate', 1.0)
        
        # Get parameter values
        self.enable_explore_lite = self.get_parameter('enable_explore_lite').get_parameter_value().bool_value
        self.free_threshold = self.get_parameter('free_threshold').get_parameter_value().integer_value
        self.occupied_threshold = self.get_parameter('occupied_threshold').get_parameter_value().integer_value
        self.use_knn = self.get_parameter('use_knn').get_parameter_value().bool_value
        self.input_map_topic = self.get_parameter('input_map_topic').get_parameter_value().string_value
        self.output_map_topic = self.get_parameter('output_map_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        self.get_logger().info(f"Explore Lite Map Converter started")
        self.get_logger().info(f"enable_explore_lite: {self.enable_explore_lite}")
        self.get_logger().info(f"free_threshold: {self.free_threshold} (prob <= 0.{self.free_threshold/100:.2f})")
        self.get_logger().info(f"occupied_threshold: {self.occupied_threshold} (prob >= 0.{self.occupied_threshold/100:.2f})")
        self.get_logger().info(f"use_knn: {self.use_knn}")
        self.get_logger().info(f"input_map_topic: {self.input_map_topic}")
        self.get_logger().info(f"output_map_topic: {self.output_map_topic}")
        
        if not self.enable_explore_lite:
            self.get_logger().info("Explore lite conversion disabled. Set enable_explore_lite:=true to enable.")
            return
            
        # QoS profile for map topics (transient local for latching behavior)
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.input_map_topic,
            self.map_callback,
            map_qos
        )
        
        # Publishers
        self.map_explore_pub = self.create_publisher(
            OccupancyGrid,
            self.output_map_topic,
            map_qos
        )
        
        # Store the latest converted map
        self.latest_map = None
        
        # Timer to periodically republish the map (helps with latching)
        self.republish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.republish_map_callback
        )
        
        self.get_logger().info(f"Map converter ready - subscribed to {self.input_map_topic}")
        self.get_logger().info(f"Publishing converted maps to {self.output_map_topic}")
        self.get_logger().info("Cartographer does not publish map updates, so only full map conversion is supported")

    def convert_occupancy_data(self, data):
        """
        Two-stage conversion process:
        
        Stage 1 - Cartographer-style thresholding:
        - Values below free_threshold: FREE (0)
        - Values above occupied_threshold: OCCUPIED (100)
        - Unknown values (-1): stay UNKNOWN (-1)
        - Values between thresholds: left for Stage 2
        
        Stage 2 - KNN for intermediate values:
        - Use adjacent neighbors to decide FREE/OCCUPIED for intermediate values
        """
        if self.use_knn:
            return self._convert_two_stage_with_knn(data)
        else:
            return self._convert_cartographer_style(data)

    def _convert_cartographer_style(self, data):
        """Stage 1 only: Cartographer-style two-threshold conversion."""
        converted_data = []
        
        for cell_value in data:
            if cell_value == -1:
                # Unknown - keep as unknown
                converted_data.append(-1)
            elif cell_value < -1 or cell_value > 100:
                # Invalid value - set to occupied for safety
                converted_data.append(100)
            elif cell_value <= self.free_threshold:
                # Low probability - mark as free
                converted_data.append(0)
            elif cell_value >= self.occupied_threshold:
                # High probability - mark as occupied
                converted_data.append(100)
            else:
                # Intermediate probability - mark as unknown for now
                converted_data.append(-1)
                    
        return converted_data

    def _convert_two_stage_with_knn(self, data):
        """
        Two-stage conversion: Cartographer thresholding + KNN for intermediate values.
        """
        # We need the map dimensions for KNN - store them from the last map message
        if not hasattr(self, '_map_width') or not hasattr(self, '_map_height'):
            self.get_logger().warn("Map dimensions not available for KNN, using Cartographer-style only")
            return self._convert_cartographer_style(data)
            
        # Convert data to numpy array for easier manipulation
        data_array = np.array(data).reshape(self._map_height, self._map_width)
        
        # Stage 1: Cartographer-style thresholding
        stage1_array = np.zeros_like(data_array)
        intermediate_mask = np.zeros_like(data_array, dtype=bool)
        
        for y in range(self._map_height):
            for x in range(self._map_width):
                cell_value = data_array[y, x]
                
                if cell_value == -1:
                    # Unknown - keep as unknown
                    stage1_array[y, x] = -1
                elif cell_value < -1 or cell_value > 100:
                    # Invalid value - set to occupied for safety
                    stage1_array[y, x] = 100
                elif cell_value <= self.free_threshold:
                    # Low probability - mark as free
                    stage1_array[y, x] = 0
                elif cell_value >= self.occupied_threshold:
                    # High probability - mark as occupied
                    stage1_array[y, x] = 100
                else:
                    # Intermediate probability - mark for Stage 2 processing
                    stage1_array[y, x] = cell_value  # Keep original value temporarily
                    intermediate_mask[y, x] = True
        
        # Stage 2: KNN for intermediate values
        stage2_array = stage1_array.copy()
        
        for y in range(self._map_height):
            for x in range(self._map_width):
                if intermediate_mask[y, x]:
                    # This cell needs KNN processing
                    stage2_array[y, x] = self._convert_cell_with_adjacent_neighbors(
                        stage1_array, data_array, x, y
                    )
        
        return stage2_array.flatten().tolist()

    def _convert_cell_with_adjacent_neighbors(self, stage1_array, original_array, x, y):
        """
        Convert an intermediate cell using its 8 adjacent neighbors.
        
        Args:
            stage1_array: Array after Stage 1 conversion
            original_array: Original occupancy values
            x, y: Cell coordinates
        """
        current_value = original_array[y, x]
        
        # Get the 8 adjacent neighbors
        neighbors = self._get_adjacent_neighbors(stage1_array, x, y)
        
        # Separate neighbors by type
        free_neighbors = [n for n in neighbors if n == 0]
        occupied_neighbors = [n for n in neighbors if n == 100]
        unknown_neighbors = [n for n in neighbors if n == -1]
        intermediate_neighbors = [n for n in neighbors if 0 < n < 100]
        
        total_neighbors = len(neighbors)
        total_definite = len(free_neighbors) + len(occupied_neighbors)
        total_unknown = len(unknown_neighbors)
        
        # Decision logic based on neighborhood consensus including unknowns
        
        # Case 1: No neighbors at all (edge case)
        if total_neighbors == 0:
            middle_point = (self.free_threshold + self.occupied_threshold) / 2
            return 0 if current_value < middle_point else 100
        
        # Case 2: Mostly unknown neighbors - cells near frontiers
        unknown_ratio = total_unknown / total_neighbors
        if unknown_ratio >= 0.4:  # Near frontier - be very conservative
            # Near true frontiers - preserve boundaries, don't create artificial FREE space
            # Only convert to FREE if current value is very low
            if current_value <= self.free_threshold + 5:  # Very close to definite free
                return 0
            elif current_value >= self.occupied_threshold - 5:  # Very close to definite occupied
                return 100
            else:
                # Intermediate values near unknown - keep as unknown to preserve frontier
                return -1
        
        # Case 3: No definite neighbors, but some intermediate ones
        if total_definite == 0:
            # Use original thresholds but consider unknown influence
            middle_point = (self.free_threshold + self.occupied_threshold) / 2
            # If many unknowns, bias slightly towards FREE
            if unknown_ratio > 0.3:
                middle_point += 10  # Make it easier to become FREE
            return 0 if current_value < middle_point else 100
        
        # Case 4: We have definite neighbors - use consensus with boundary preservation
        free_ratio = len(free_neighbors) / total_definite
        
        # Special handling for boundary areas (adjacent to unknown)
        if unknown_ratio > 0.15:  # Some unknown neighbors - near boundary
            # Be more conservative near boundaries to preserve frontier integrity
            if current_value <= self.free_threshold + 5:
                return 0  # Only very low values become FREE
            elif current_value >= self.occupied_threshold - 5:
                return 100  # Only very high values become OCCUPIED
            else:
                # Intermediate values near boundaries - preserve as unknown for frontier detection
                return -1
        
        # Standard consensus for interior areas (away from frontiers)
        # Use stricter thresholds for cleaner conversion
        if current_value <= (self.free_threshold + self.occupied_threshold) / 2:
            # Lower half - easier to become free
            return 0 if free_ratio >= 0.4 else 100
        else:
            # Upper half - easier to become occupied  
            return 0 if free_ratio >= 0.7 else 100

    def _get_adjacent_neighbors(self, data_array, center_x, center_y):
        """
        Get the 8 adjacent neighbors (Moore neighborhood).
        
        Args:
            data_array: 2D numpy array of the occupancy grid
            center_x, center_y: Center cell coordinates
            
        Returns:
            List of neighbor values
        """
        neighbors = []
        height, width = data_array.shape
        
        # Define the 8 adjacent directions
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        for dx, dy in directions:
            nx = center_x + dx
            ny = center_y + dy
            
            # Check bounds
            if 0 <= nx < width and 0 <= ny < height:
                neighbors.append(data_array[ny, nx])
                    
        return neighbors

    def _get_conversion_stats(self, data):
        """Get statistics about map data for logging."""
        if not data:
            return "empty"
            
        data_array = np.array(data)
        free_count = np.sum(data_array == 0)
        occupied_count = np.sum(data_array == 100)
        unknown_count = np.sum(data_array == -1)
        invalid_count = np.sum((data_array < -1) | (data_array > 100))
        other_count = len(data) - free_count - occupied_count - unknown_count - invalid_count
        
        return f"FREE:{free_count}, OCC:{occupied_count}, UNK:{unknown_count}, INV:{invalid_count}, OTHER:{other_count}"

    def map_callback(self, msg):
        """Handle incoming map messages."""
        self.get_logger().debug("Received full map update")
        
        # Store map dimensions for KNN processing
        self._map_width = msg.info.width
        self._map_height = msg.info.height
        
        # Create a copy of the message
        converted_msg = OccupancyGrid()
        converted_msg.header = msg.header
        converted_msg.info = msg.info
        
        # Convert the occupancy data
        converted_data = self.convert_occupancy_data(msg.data)
        converted_msg.data = converted_data
        
        # Log conversion statistics
        original_stats = self._get_conversion_stats(msg.data)
        converted_stats = self._get_conversion_stats(converted_data)
        
        conversion_method = "Two-stage (Cartographer + KNN)" if self.use_knn else "Cartographer-style"
        self.get_logger().info(
            f"{conversion_method} (free<={self.free_threshold}, occ>={self.occupied_threshold}) conversion - "
            f"Original: {original_stats}, Converted: {converted_stats}"
        )
        
        # Store for republishing
        self.latest_map = converted_msg
        
        # Publish immediately
        self.map_explore_pub.publish(converted_msg)

    def map_updates_callback(self, msg):
        """Handle incoming map update messages."""
        self.get_logger().debug("Received map update")
        
        # Create a copy of the message
        converted_msg = OccupancyGridUpdate()
        converted_msg.header = msg.header
        converted_msg.x = msg.x
        converted_msg.y = msg.y
        converted_msg.width = msg.width
        converted_msg.height = msg.height
        
        # For map updates, try to use KNN if we have map context and the update is small
        # For large updates or when no context, fall back to Cartographer-style
        if (self.use_knn and 
            hasattr(self, '_map_width') and hasattr(self, '_map_height') and
            hasattr(self, 'latest_map') and self.latest_map is not None and
            msg.width * msg.height < 100):  # Small update that can benefit from KNN
            
            converted_data = self._convert_map_update_with_context(msg)
        else:
            # Large update or no context - use Cartographer-style
            converted_data = self._convert_cartographer_style(msg.data)
            
        converted_msg.data = converted_data
        
        # Publish the converted update
        self.map_updates_explore_pub.publish(converted_msg)

    def _convert_map_update_with_context(self, update_msg):
        """
        Convert map update using context from the latest full map when possible.
        """
        if self.latest_map is None:
            return self._convert_cartographer_style(update_msg.data)
            
        # Get the latest converted map data
        latest_map_array = np.array(self.latest_map.data).reshape(self._map_height, self._map_width)
        
        # Process each cell in the update
        converted_data = []
        
        for i, cell_value in enumerate(update_msg.data):
            # Calculate position in the update
            local_y = i // update_msg.width
            local_x = i % update_msg.width
            
            # Calculate global position
            global_x = update_msg.x + local_x
            global_y = update_msg.y + local_y
            
            # Check bounds
            if (0 <= global_x < self._map_width and 0 <= global_y < self._map_height):
                # Use KNN with context from latest map
                converted_value = self._convert_update_cell_with_context(
                    cell_value, latest_map_array, global_x, global_y
                )
            else:
                # Out of bounds - use simple conversion
                if cell_value == -1:
                    converted_value = -1
                elif cell_value <= self.free_threshold:
                    converted_value = 0
                elif cell_value >= self.occupied_threshold:
                    converted_value = 100
                else:
                    converted_value = -1  # Conservative for boundaries
            
            converted_data.append(converted_value)
            
        return converted_data

    def _convert_update_cell_with_context(self, cell_value, context_map, x, y):
        """
        Convert a single update cell using context from the latest map.
        """
        # Handle definite cases first
        if cell_value == -1:
            return -1
        elif cell_value <= self.free_threshold:
            return 0
        elif cell_value >= self.occupied_threshold:
            return 100
            
        # For intermediate values, use neighborhood from context map
        neighbors = self._get_adjacent_neighbors(context_map, x, y)
        
        if not neighbors:
            # No neighbors - use conservative conversion
            return -1 if cell_value < 50 else 100
            
        # Count neighbor types
        free_neighbors = len([n for n in neighbors if n == 0])
        occupied_neighbors = len([n for n in neighbors if n == 100])
        unknown_neighbors = len([n for n in neighbors if n == -1])
        
        total_neighbors = len(neighbors)
        unknown_ratio = unknown_neighbors / total_neighbors
        
        # Near boundaries - be conservative
        if unknown_ratio > 0.2:
            return -1  # Preserve frontier integrity
            
        # Interior area - use consensus
        total_definite = free_neighbors + occupied_neighbors
        if total_definite > 0:
            free_ratio = free_neighbors / total_definite
            return 0 if free_ratio > 0.5 else 100
        else:
            # Only unknown neighbors
            return -1

    def republish_map_callback(self):
        """Periodically republish the latest map to ensure it's available."""
        if self.latest_map is not None:
            self.map_explore_pub.publish(self.latest_map)


def main(args=None):
    rclpy.init(args=args)
    
    node = ExploreLiteMapConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
