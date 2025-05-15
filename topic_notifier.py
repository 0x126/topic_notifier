#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import re
import yaml
from typing import List, Dict, Tuple, Pattern, Any
import os
import sys

from PySide2 import QtCore
from PySide2.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QScrollArea, QFrame, QSizePolicy, QGridLayout
)
from PySide2.QtCore import Qt
from PySide2.QtGui import QColor, QPalette


class TopicStatusWidget(QFrame):
    """Widget to display the status of a single topic."""
    
    def __init__(self, topic_name, frequency, expected_range, status, parent=None):
        super().__init__(parent)
        self.topic_name = topic_name
        self.frequency = frequency
        self.expected_range = expected_range
        self.status = status
        
        self.setupUI()
        self.updateStatus(status)
        
    def setupUI(self):
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMinimumHeight(80)
        
        layout = QVBoxLayout(self)
        
        # Topic name
        self.topic_label = QLabel(self.topic_name)
        self.topic_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.topic_label)
        
        # Frequency and expected range
        info_layout = QHBoxLayout()
        self.freq_label = QLabel(f"Frequency: {self.frequency:.2f} Hz")
        self.range_label = QLabel(f"Expected: {self.expected_range}")
        info_layout.addWidget(self.freq_label)
        info_layout.addWidget(self.range_label)
        layout.addLayout(info_layout)
        
        # Status
        self.status_label = QLabel(f"Status: {self.status}")
        layout.addWidget(self.status_label)
        
    def updateStatus(self, status):
        self.status = status
        self.status_label.setText(f"Status: {self.status}")
        
        # Set background color based on status
        palette = self.palette()
        if status == 'NG':
            palette.setColor(QPalette.Window, QColor(255, 200, 200))  # Light red
        elif status == 'OK':
            palette.setColor(QPalette.Window, QColor(200, 255, 200))  # Light green
        else:
            palette.setColor(QPalette.Window, QColor(255, 255, 200))  # Light yellow
        
        self.setAutoFillBackground(True)
        self.setPalette(palette)
        
    def updateFrequency(self, frequency):
        self.frequency = frequency
        self.freq_label.setText(f"Frequency: {self.frequency:.2f} Hz")


class TopicNotifierGUI(QMainWindow):
    """Main window for the Topic Notifier application."""
    
    def __init__(self):
        super().__init__()
        self.topic_widgets = {}  # Dictionary to store topic widgets
        self.setupUI()
        self.should_exit = False  # Flag to indicate if the application should exit
        
    def setupUI(self):
        self.setWindowTitle("ROS2 Topic Frequency Monitor")
        self.resize(800, 800)
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
        
        # 画面の右上端にウィンドウを配置
        screen_geometry = QApplication.screens()[0].availableGeometry()
        window_geometry = self.frameGeometry()
        window_geometry.moveTopRight(screen_geometry.topRight())
        self.move(window_geometry.topLeft())
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Header
        header_layout = QHBoxLayout()
        title_label = QLabel("Topic Frequency Monitor")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        header_layout.addWidget(title_label)
        
        # Clear button
        self.clear_button = QPushButton("Clear All")
        self.clear_button.clicked.connect(self.clearAllTopics)
        header_layout.addWidget(self.clear_button)
        
        main_layout.addLayout(header_layout)
        
        # Scroll area for topics
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        main_layout.addWidget(scroll_area)
        
        # Container for topic widgets
        self.topics_container = QWidget()
        self.topics_layout = QVBoxLayout(self.topics_container)
        scroll_area.setWidget(self.topics_container)
        
        # Status bar
        self.statusBar().showMessage("Monitoring topics...")
        
    def closeEvent(self, event):
        """Handle window close event."""
        self.should_exit = True
        event.accept()
        
    def clearAllTopics(self):
        """Clear all topic widgets."""
        for widget in self.topic_widgets.values():
            widget.setParent(None)
        self.topic_widgets.clear()
        
    def updateTopicStatus(self, topic_name, frequency, expected_range, status):
        """Update or add a topic status widget."""
        # Only display topics with NG status
        if status == 'NG':
            if topic_name in self.topic_widgets:
                # Update existing widget
                widget = self.topic_widgets[topic_name]
                widget.updateFrequency(frequency)
                widget.updateStatus(status)
            else:
                # Create new widget
                widget = TopicStatusWidget(topic_name, frequency, expected_range, status)
                self.topic_widgets[topic_name] = widget
                self.topics_layout.addWidget(widget)
        elif topic_name in self.topic_widgets:
            # Remove widget if status changed from NG to something else
            widget = self.topic_widgets[topic_name]
            widget.setParent(None)
            del self.topic_widgets[topic_name]
        
        # Update status bar
        ng_count = len(self.topic_widgets)
        self.statusBar().showMessage(f"Monitoring topics... {ng_count} issues found")


class TopicNotifier(Node):
    """
    ROS2 node that monitors topic frequencies and displays notifications in a GUI.
    """

    def __init__(self, gui):
        super().__init__('topic_notifier')
        
        # Store reference to GUI
        self.gui = gui
        
        # Declare parameters
        self.declare_parameter('yaml_config_files', ['/opt/drs/record_topics_ecu0.yaml', '/opt/drs/record_topics_ecu1.yaml'])
        self.declare_parameter('target_node_name', 'rosbag_checker_live')
        
        # Get parameters
        yaml_filepaths = self.get_parameter('yaml_config_files').value
        self.target_node_name = self.get_parameter('target_node_name').value
        
        # Load configuration from YAML files
        self.exact_hz_configs, self.regex_hz_configs = self.load_hz_range_config_from_files(yaml_filepaths)
        
        # Log loaded configurations
        self.log_configurations()
        
        # Create subscription to /rosout
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            10  # QoS profile depth
        )
        
        self.get_logger().info('Topic notifier started. Listening for /rosout messages...')

    def rosout_callback(self, msg: Log):
        """
        Callback for /rosout messages.
        Processes messages from the target node containing topic statistics.
        """
        # Check if the message is from our target node
        if self.target_node_name in msg.name:
            self.get_logger().info(f"Received message from {msg.name}")
            
            # Parse the topic statistics from the message
            parsed_topic_stats = self.parse_ros_topic_statistics(msg.msg)
            
            if parsed_topic_stats:
                # Validate the topic frequencies
                validation_results = self.check_topic_frequencies(
                    parsed_topic_stats, 
                    self.exact_hz_configs, 
                    self.regex_hz_configs
                )
                
                # Update GUI with results
                for result in validation_results:
                    self.gui.updateTopicStatus(
                        result['topic'],
                        result['frequency'],
                        result['expected_range'],
                        result['status']
                    )
                
                # Log the results
                self.log_validation_results(validation_results)
            else:
                self.get_logger().info("No topic statistics found in the message")

    def parse_ros_topic_statistics(self, msg_str: str) -> List[Dict[str, Any]]:
        """
        Parse ROS topic statistics from a string message.
        """
        ansi_escape_pattern = re.compile(r'\x1B\[[0-?]*[ -/]*[@-~]')
        cleaned_str = ansi_escape_pattern.sub('', msg_str)
        
        # Modified pattern to handle 'nan' values
        block_pattern = re.compile(
            r"Statistics for topic (.+?)\n"
            r"Message count = (\d+), Message frequency = ([\d.]+|nan)"
        )
        
        self.get_logger().debug(f"Parsing message: {cleaned_str}")
        
        results = []
        for match in block_pattern.finditer(cleaned_str):
            topic_name = match.group(1).strip()
            message_count = int(match.group(2))
            
            # Handle 'nan' frequency
            freq_str = match.group(3)
            if freq_str == 'nan':
                message_frequency = float('nan')
            else:
                message_frequency = float(freq_str)
            
            self.get_logger().debug(f"Found topic: {topic_name}, count: {message_count}, freq: {message_frequency}")
            
            results.append({
                'topic': topic_name,
                'message_count': message_count,
                'message_frequency': message_frequency
            })
        
        if not results:
            self.get_logger().debug(f"No matches found in message. Regex pattern: {block_pattern.pattern}")
            # Log a sample of the message to help with debugging
            if len(cleaned_str) > 200:
                self.get_logger().debug(f"Message sample: {cleaned_str[:200]}...")
            else:
                self.get_logger().debug(f"Message: {cleaned_str}")
        
        return results

    def load_hz_range_config_from_files(self, yaml_filepaths_list: List[str]) -> Tuple[Dict[str, Dict[str, float]], List[Tuple[Pattern, Dict[str, float]]]]:
        """
        Load topic HZ range settings from multiple YAML files.
        Handles topic names with regular expressions.
        """
        exact_match_configs = {}
        regex_configs = []

        for filepath in yaml_filepaths_list:
            try:
                # Try to find the file in the package directory if it's not an absolute path
                if not os.path.isabs(filepath):
                    # Check current directory first
                    if os.path.exists(filepath):
                        full_path = filepath
                    else:
                        # Could add package path resolution here if needed
                        full_path = filepath
                else:
                    full_path = filepath
                
                with open(full_path, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    if data and 'topics' in data:
                        for topic_entry in data['topics']:
                            name = topic_entry.get('name')
                            hz_range_list = topic_entry.get('hz_range')

                            if name and hz_range_list and len(hz_range_list) == 2:
                                try:
                                    config = {'min': float(hz_range_list[0]), 'max': float(hz_range_list[1])}
                                    # Check if the name contains regex patterns
                                    if any(char in name for char in ['*', '+', '?', '^', '$', '[', ']', '(', ')', '{', '}', '|', '\\']):
                                        regex_pattern = re.compile(name)
                                        regex_configs.append((regex_pattern, config))
                                    else:
                                        exact_match_configs[name] = config
                                except ValueError:
                                    self.get_logger().warning(f"Invalid hz_range format for topic '{name}' in file '{filepath}': {hz_range_list}")
                                except re.error as e:
                                    self.get_logger().warning(f"Invalid regex pattern for topic '{name}' in file '{filepath}': {e}")
            except FileNotFoundError:
                self.get_logger().error(f"YAML file not found at '{filepath}'")
            except yaml.YAMLError as e:
                self.get_logger().error(f"Error parsing YAML file '{filepath}': {e}")
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred while processing YAML file '{filepath}': {e}")

        return exact_match_configs, regex_configs

    def check_topic_frequencies(self, parsed_stats: List[Dict[str, Any]], 
                               exact_match_configs: Dict[str, Dict[str, float]], 
                               regex_configs_list: List[Tuple[Pattern, Dict[str, float]]]) -> List[Dict[str, Any]]:
        """
        Compare parsed topic statistics with HZ range settings and return results.
        """
        validation_results = []
        for stat in parsed_stats:
            topic_name = stat['topic']
            frequency = stat['message_frequency']
            
            found_config = None
            config_source_info = "N/A"

            if topic_name in exact_match_configs:
                found_config = exact_match_configs[topic_name]
                config_source_info = f"Exact match: {topic_name}"
            else:
                for pattern, config in regex_configs_list:
                    if pattern.fullmatch(topic_name):  # Use fullmatch to match the entire string
                        found_config = config
                        config_source_info = f"Regex match: {pattern.pattern}"
                        break
            
            result_entry = {
                'topic': topic_name,
                'frequency': frequency,
                'expected_range': "N/A",
                'status': 'Config Not Found',
                'config_source': config_source_info if found_config else "N/A"
            }

            if found_config:
                result_entry['expected_range'] = f"[{found_config['min']}, {found_config['max']}]"
                if found_config['min'] <= frequency <= found_config['max']:
                    result_entry['status'] = 'OK'
                else:
                    result_entry['status'] = 'NG'
            
            validation_results.append(result_entry)
            
        return validation_results

    def log_configurations(self):
        """Log the loaded HZ range configurations."""
        self.get_logger().info("--- Loaded HZ Range Configurations ---")
        if self.exact_hz_configs or self.regex_hz_configs:
            self.get_logger().info("Exact Match Configs:")
            for topic, conf in self.exact_hz_configs.items():
                self.get_logger().info(f"  '{topic}': {conf}")
            self.get_logger().info("Regex Match Configs:")
            for pattern, conf in self.regex_hz_configs:
                self.get_logger().info(f"  '{pattern.pattern}': {conf}")
        else:
            self.get_logger().info("No HZ range configurations were loaded.")

    def log_validation_results(self, validation_results: List[Dict[str, Any]]):
        """Log the topic frequency validation results."""
        self.get_logger().info("--- Topic Frequency Validation Results ---")
        for result in validation_results:
            status_str = result['status']
            message = (
                f"Topic: {result['topic']}\n"
                f"  Frequency: {result['frequency']}\n"
                f"  Expected Range: {result['expected_range']}\n"
                f"  Status: {status_str}\n"
                f"  Config Source: {result['config_source']}"
            )
            
            # Use different severity levels based on status
            if status_str == 'NG':
                self.get_logger().error(message)
            elif status_str == 'Config Not Found':
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)


def main(args=None):
    # Initialize Qt application
    app = QApplication(sys.argv)
    
    # Create GUI
    gui = TopicNotifierGUI()
    gui.show()
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create node with reference to GUI
    node = TopicNotifier(gui)
    
    # Main event loop that processes both Qt and ROS events
    try:
        while not gui.should_exit:
            app.processEvents()  # Process Qt events
            rclpy.spin_once(node, timeout_sec=0.01)  # Process ROS events with timeout
    finally:
        # Clean up ROS resources when exiting
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
