# hewo_face_pkg

`hewo_face_pkg` is the **ROS 2 wrapper package** for the visual module [`hewo-face`](https://github.com/Daiego43/hewo_face), responsible for rendering and publishing HeWo's emotional and graphical state into the ROS ecosystem.

It depends on:

- [`hewo-face`](https://github.com/Daiego43/hewo_face): visual logic and animations using Pygame
- [`hewo_face_interfaces`](https://github.com/Daiego43/hewo_face_interfaces): custom ROS 2 message definitions

---

## 📦 Installation

This package **does not contain the display logic itself** — it simply wraps the Python `hewo-face` module for ROS 2 integration.

Before building, install the visual module:

```bash
pip install hewo-face
````

Then build this ROS package using `colcon` inside your workspace.

---

## 🚀 Main Node

The `hewo_main_node` launches the HeWo face interface and communicates through the following ROS topics:

### 🔊 Publishers

| Topic            | Type                               | Description                 |
| ---------------- | ---------------------------------- | --------------------------- |
| `/hewo/emotion`  | `hewo_face_interfaces/msg/Emotion` | Current emotional state     |
| `/hewo/position` | `geometry_msgs/msg/Point`          | Face position (x, y)        |
| `/hewo/size`     | `geometry_msgs/msg/Vector3`        | Current face size           |
| `/hewo/frame`    | `sensor_msgs/msg/Image`            | RGB frame output (optional) |

### 🎧 Subscribers

| Topic                        | Type                                          | Action                           |
| ---------------------------- | --------------------------------------------- | -------------------------------- |
| `/hewo/set_emotion_goal`     | `hewo_face_interfaces/msg/Emotion`            | Sets the target emotion          |
| `/hewo/adjust_emotion_point` | `hewo_face_interfaces/msg/AdjustEmotionPoint` | Adjusts a specific emotion point |
| `/hewo/adjust_position`      | `hewo_face_interfaces/msg/AdjustPosition`     | Adjusts face position (dx, dy)   |
| `/hewo/set_size`             | `std_msgs/msg/UInt32`                         | Sets face size                   |
| `/hewo/trigger_blink`        | `std_msgs/msg/Empty`                          | Triggers a blink                 |
| `/hewo/toggle_talk`          | `std_msgs/msg/Empty`                          | Toggles talk animation           |

---

## 🧪 Test Node

The package also includes `hewo_test_node`, which:

* Subscribes to the face status topics
* Publishes test data (emotion, position, size, talk)

To run:

```bash
ros2 run hewo_face_pkg hewo_test_node
```

---

## 🎮 Runtime Notes

This package requires graphical display access to render the face using Pygame.
Make sure to configure environment variables such as `DISPLAY` or `SDL_VIDEODRIVER` appropriately in embedded or headless environments.

---

## 🧼 Author

Developed by [Diego Delgado Chaves](mailto:diedelcha@gmail.com)
License: MIT