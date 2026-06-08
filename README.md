<p align="center">
  <!-- ---------- ARISE logo ---------- -->
  <!-- Light mode -->
  <img src="repo_images/ARISE_logo-light_mode.png#gh-light-mode-only" alt="Logo for light mode" height="60"/>
  <!-- Dark mode -->
  <img src="repo_images/ARISE_logo-dark_mode.png#gh-dark-mode-only" alt="Logo for dark mode" height="60"/>
</p>


<p align="center">
  These 
  Modules are part of the ARISE Middleware<br>
  <a href="https://arise-middleware.eu/">ARISE Middleware site</a>
</p>



[![vulcanexus](https://img.shields.io/badge/Vulcanexus%20Version-Jazzy-%230895CD?style=flat)](https://github.com/eProsima/vulcanexus)
[![ubuntu24](https://img.shields.io/badge/Ubuntu-24.04-%23E95420?style=flat&logo=ubuntu
)](https://releases.ubuntu.com/24.04/)
---

# PCB component Detection Module

This repo provides a reusable ROS 2 and FIWARE-based module for detecting Integrated Circuits (ICs) on Printed Circuit Boards (PCBs), publishing the resulting inspection metadata, and visualizing the detected results on a Grafana dashboard.

The module is focused on a inspection workflow:

1. An RGB frame of a PCB is received as input.
2. A ROS 2 detection service sends the frame to a YOLO-based IC detection server.
3. The detection server returns the detected IC location and an annotated image.
4. The annotated image is uploaded to a FastAPI image server.
5. The PCB metadata is published through ROS 2/DDS.
6. A DDS-to-NGSI-LD translation layer maps the ROS 2 message into an NGSI-LD entity.
7. FIWARE Orion-LD stores the PCB context data.
8. Grafana displays the detected IC result and the annotated PCB image.

This structure makes the IC detection logic reusable in other PCB inspection stations while keeping the image payload, semantic metadata, and dashboard visualization cleanly separated.

---

## General Overview

The module implementation is organized as a perception-to-dashboard pipeline for PCB IC detection. The core detection component is implemented as a ROS 2 service. The service server runs the IC detection model, receives an RGB PCB frame, detects the target IC/component, and returns the annotated detection result.

Large image payloads are not sent directly to the context broker. Instead, the annotated PCB image is uploaded to a FastAPI image server, and only the resulting image URL plus structured PCB metadata are forwarded to the broker pipeline.

The broker-side integration is enabled by FIWARE Orion-LD and a DDS-to-NGSI-LD message translation layer. This allows ROS 2 inspection results to be represented as NGSI-LD context entities and visualized in Grafana.

The module includes:

- ROS 2 service-based IC detection on RGB PCB frames.
- YOLO-based detection server for identifying ICs or selected PCB components.
- FastAPI image server for storing and serving annotated PCB images.
- ROS 2 custom interfaces for detection requests, detection responses, and PCB metadata.
- DDS-to-NGSI-LD message translation for broker integration.
- FIWARE Orion-LD context broker for storing PCB inspection context.
- TimescaleDB/Mintaka support for temporal context storage.
- Grafana dashboard for displaying the latest PCB IC detection result.

---

## Architecture

```text
RGB PCB Frame
     |
     v
ROS 2 Detection Client
     |
     | ComponentDetection.srv request
     v
ROS 2 Detection Server
YOLO IC Detection Model
     |
     | annotated frame + IC coordinates
     v
PCB Metadata Publisher
     |
     | uploads image
     v
FastAPI Image Server
     |
     | image URL
     v
ROS 2 / DDS Metadata Topic
     |
     | DDS-to-NGSI-LD translation
     v
FIWARE Orion-LD Context Broker
     |
     v
Temporal Storage / Mintaka / TimescaleDB
     |
     v
Grafana Dashboard
```

<p align="center">
  <img src="repo_images/SF_architecture.png" width="540" heigth="250"/></a>
</p>
The main design decision is to separate the annotated image from the semantic metadata. The broker receives lightweight structured data, while Grafana uses the stored image URL to display the detection result.

---

## Features

- IC detection on PCB RGB frames.
- ROS 2 service interface for detection requests.
- Detection server running the trained YOLO model.
- Annotated PCB image generation.
- Image upload through a FastAPI server.
- Broker-friendly PCB metadata publication.
- DDS-to-NGSI-LD message translation.
- FIWARE Orion-LD context broker integration.
- Grafana dashboard visualization of PCB metadata and annotated images.
- Reusable package structure for adapting the module to other PCB inspection tasks.

---

## Repository Layout

```text
PrepStation/
├── build/
│   └── custom_vulcanexus/
│       └── Dockerfile
├── configs/
│   ├── contextbroker/
│   ├── grafanadashboard/
│   └── ros2_pkgs/
│       └── control_station_ws/
│           └── src/
│               ├── communication_interfaces/
│               ├── defective_pcb_detector/
│               └── pcb_img_server/
├── docker-compose.yaml
└── readme.md
```

---

## Main Packages

### `defective_pcb_detector`

This package contains the IC detection workflow. It includes the ROS 2 nodes used for frame acquisition, service-based detection, annotated result handling, and metadata publication.

Main executables:

| Script | Purpose |
| --- | --- |
| `yolo_detection_server` | Runs the ROS 2 service server with the YOLO IC detection model. It receives an RGB PCB frame and returns detected IC coordinates plus an annotated frame. |
| `wrist_camera_publisher` | Publishes RGB camera frames from the workstation or wrist-mounted camera. |
| `component_selection_client` | Sends the selected component/IC class and RGB frame to the detection service. |
| `defective_pcb_to_server` | Uploads the annotated image to the FastAPI server and publishes the PCB metadata message. |

Although the package name contains `defective`, the current README frames the module as IC detection on PCBs. The same structure can later be extended to defect detection, missing-component detection, or assembly verification.

---

### `communication_interfaces`

This package defines the custom ROS 2 messages and services used between the detection, image-upload, and broker-publication parts of the system.

```text
msg/PcbInfoMsg.msg
msg/PcbMetaMsg.msg
msg/Componentdata.msg
srv/ComponentDetection.srv
```

These interfaces keep the module reusable. The detection service handles image-based inference, while the metadata message exposes only the values required by the broker and dashboard.

---

### `pcb_img_server`

This package provides the FastAPI server used to store and serve annotated PCB images.

The image server avoids sending heavy image payloads to Orion-LD. The annotated detection frame is uploaded to the server, and the resulting URL is inserted into the PCB metadata entity.

Runtime command:

```bash
python3 -m uvicorn pcb_img_server.run_server:app --host 0.0.0.0 --port 5000
```

---

## Data Flow

### 1. RGB frame input

The pipeline starts with a PCB RGB frame. The frame may come from a camera publisher, a workstation camera, a robot-mounted camera, or any ROS 2 node capable of publishing or providing `sensor_msgs/Image` frames.

```text
Input: RGB PCB frame
Type: sensor_msgs/Image
```

---

### 2. ROS 2 detection service request

The detection client sends the RGB frame to the detection server using `ComponentDetection.srv`.

Request fields:

| Field | Type | Description |
| --- | --- | --- |
| `component_id` | `uint8` | Target IC/component class to detect. |
| `raw_frame` | `sensor_msgs/Image` | RGB frame of the PCB. |

---

### 3. Detection server response

The detection server runs the YOLO-based IC detection model and returns the annotated frame and detected coordinates.

Response fields:

| Field | Type | Description |
| --- | --- | --- |
| `annotated_frame` | `sensor_msgs/Image` | RGB PCB frame with IC detections drawn on it. |
| `location_x` | `float64[]` | X coordinates of detected IC/component centers. |
| `location_y` | `float64[]` | Y coordinates of detected IC/component centers. |
| `component_class` | `uint8[]` | Detected IC/component class IDs. |

<p align="center">
  <!-- ---------- ARISE logo ---------- -->
  <!-- Light mode -->
  <img src="repo_images/gui.png"  width= "440" heigth="250"/>
</p>

---

### 4. Annotated image upload

After detection, the annotated image is uploaded to the FastAPI image server. The image server returns a URL.

```text
Annotated frame -> FastAPI image server -> image URL
```

The URL is used by Grafana to render the detected PCB image in the dashboard.

---

### 5. PCB metadata publication

The metadata publisher creates a lightweight ROS 2 message containing the PCB context information and the image URL.

`PcbMetaMsg` fields:

| Field | Type | Description |
| --- | --- | --- |
| `id` | `string` | PCB or inspection identifier. |
| `departured` | `string` | Timestamp or station status information. |
| `width` | `float32` | PCB image width. |
| `height` | `float32` | PCB image height. |
| `defected` | `bool` | Detection/status flag. For IC detection, this can represent whether the target IC was detected. |
| `material_info` | `string` | PCB material or board metadata. |
| `heatsink_number` | `int32` | Additional board/component descriptor retained from the current interface. |
| `component_class` | `uint8` | Detected IC/component class. |
| `defect_loc_x` | `float64` | X coordinate of the detected IC/component. |
| `defect_loc_y` | `float64` | Y coordinate of the detected IC/component. |
| `url` | `string` | URL of the annotated PCB image on the FastAPI image server. |

---

### 6. DDS-to-NGSI-LD translation

The ROS 2 metadata message is transported through DDS and translated into NGSI-LD. The translated entity is then sent to Orion-LD.

Conceptually:

```text
ROS 2 / DDS message
     -> DDS-to-NGSI-LD mapping
     -> NGSI-LD PCB entity
     -> Orion-LD context broker
```

A typical NGSI-LD entity can be represented as:

```text
Entity ID: urn:ngsi-ld:pcb:1
Entity Type: PCB
Attribute: mypcb
```

The NGSI-LD entity stores the image URL, detected IC coordinates, component class, frame dimensions, and other PCB metadata.

---

### 7. Grafana dashboard visualization

Grafana reads the stored PCB metadata and displays the result as an inspection panel. The panel can show:

- PCB identifier.
- Detection status.
- Detected IC/component class.
- IC location coordinates.
- Image width and height.
- Timestamp/status field.
- Annotated PCB image loaded from the FastAPI image URL.

The resulting dashboard view shows the detected model output for the incoming RGB frame, enabled by Orion-LD, temporal storage, and the DDS-to-NGSI-LD message translation layer.

<p align="center">
  <img src="repo_images/grafanalive.png"  width= "440" heigth="250"/>
</p>

---

## Running the Module

### 1. Build the stack

From the repository root:

```bash
sudo docker compose build
```

### 2. Start the services

```bash
sudo docker compose up
```

### 3. Source the ROS 2 workspace

Inside the ROS 2 container:

```bash
sudo docker exec -it pcbinfo bash
source /opt/vulcanexus/jazzy/setup.bash
source /control_station_ws/install/setup.bash
```

### 4. Start the FastAPI image server

```bash
sudo docker exec -it pcbinfo bash
python3 -m uvicorn pcb_img_server.run_server:app --host 0.0.0.0 --port 5000
```

### 5. Start the IC detection service server

```bash
sudo docker exec -it pcbinfo bash
ros2 run defective_pcb_detector yolo_detection_server
```

This server waits for `ComponentDetection.srv` requests. Each request contains an RGB PCB frame and a selected IC/component class.

### 6. Start the RGB frame publisher
In a new termianl, run the ROS2 node that publisher the rgb frame from your camera, for example:  

```bash
sudo docker exec -it pcbinfo bash
ros2 run defective_pcb_detector wrist_camera_publisher
```


### 7. Start the detection client

```bash
sudo docker exec -it pcbinfo bash
ros2 run defective_pcb_detector component_selection_client
```

The client sends the RGB frame to the detection service and receives the annotated result.

### 8. Start the metadata and image-upload node

```bash
sudo docker exec -it pcbinfo bash
ros2 run defective_pcb_detector defective_pcb_to_server
```

This node uploads the annotated image to the FastAPI server and publishes the lightweight metadata message that is translated into NGSI-LD for Orion-LD.

---

## Orion-LD Context Broker Integration

The module uses Orion-LD as the NGSI-LD context broker. The broker receives translated PCB metadata from the ROS 2/DDS layer.

The broker should receive lightweight structured context data instead of raw images. Therefore, the annotated image is stored on the FastAPI server, and the NGSI-LD entity only stores the image URL.

Example broker-oriented metadata:

```text
id: urn:ngsi-ld:pcb:1
type: PCB
mypcb.url: http://<image-server-host>:5000/<annotated-image>
mypcb.component_class: detected IC/component class
mypcb.defect_loc_x: detected X coordinate
mypcb.defect_loc_y: detected Y coordinate
mypcb.width: RGB frame width
mypcb.height: RGB frame height
```

This keeps the context broker responsible for semantic state, while the image server is responsible for binary image delivery.

---

## Grafana Visualization

Grafana is used to display the PCB IC detection result. The dashboard queries the stored NGSI-LD temporal data and renders both the metadata and the annotated image.

Grafana endpoint:

```text
http://localhost:443/
```

Default credentials:

```text
admin / admin
```

A sample query for retrieving the latest PCB IC detection result is:

```sql
SELECT
  ts AS "time",
  compound->>'id' AS pcb_id,
  compound->>'url' AS image_url,
  (compound->>'width')::int AS width,
  (compound->>'height')::int AS height,
  (compound->>'defected')::boolean AS detected,
  (compound->>'component_class')::int AS component_class,
  (compound->>'defect_loc_x')::float AS x,
  (compound->>'defect_loc_y')::float AS y,
  compound->>'material_info' AS material_info,
  compound->>'departured' AS departured
FROM
  attributes
WHERE
  entityId = 'urn:ngsi-ld:pcb:1'
  AND id = 'https://uri.etsi.org/ngsi-ld/default-context/mypcb'
ORDER BY
  ts DESC
LIMIT 1;
```

Example Grafana HTML panel:

```html
<h2>PCB IC Detection Result</h2>
<p><strong>PCB ID:</strong> {{@root.pcb_id}}</p>
<p><strong>IC detected:</strong> {{@root.detected}}</p>
<p><strong>Component class:</strong> {{@root.component_class}}</p>
<p><strong>Location:</strong> ({{@root.x}}, {{@root.y}})</p>
<p><strong>Frame size:</strong> {{@root.width}} × {{@root.height}}</p>
<p><strong>Material:</strong> {{@root.material_info}}</p>
<p><strong>Timestamp/status:</strong> {{@root.departured}}</p>

<figure style="text-align: center;">
  <img src="{{{ @root.image_url }}}" alt="Annotated PCB IC detection result" width="400" style="border: 1px solid #ccc;" />
  <figcaption style="font-size: 14px; color: #666; margin-top: 8px;">
    Detected IC result generated from the input RGB PCB frame.
  </figcaption>
</figure>
```

---

## Reusable Module Adaptation

This repository can be reused as a template for vision-to-context-broker applications.

To adapt the module:

1. Replace or retrain the YOLO model for the target PCB components.
2. Update the component class mapping used by the detection client and server.
3. Modify `ComponentDetection.srv` if a different detection request or response is needed.
4. Extend `PcbMetaMsg.msg` if additional dashboard fields are required.
5. Update the DDS-to-NGSI-LD mapping for the new ROS 2 topic or entity type.
6. Update the Orion-LD entity type and attribute naming.
7. Update the Grafana SQL query and dashboard panel.

The same structure can support other PCB inspection use cases, including:

- IC detection.
- Missing IC detection.
- Connector detection.
- Solder-joint localization.
- Board assembly verification.
- Operator or robot-assisted rework guidance.

---

## Summary

PrepStation implements a reusable PCB IC detection module where the detection model runs behind a ROS 2 service. An RGB frame is passed to the detection server, the detected IC result is annotated, the annotated image is uploaded to a FastAPI image server, and the resulting metadata is translated from DDS to NGSI-LD and sent to Orion-LD. Grafana then displays the latest PCB detection result using the broker-enabled metadata and the stored image URL.

---

## License

This repository is distributed under the GPL-3.0 license. Individual ROS 2 packages may include their own package-level license metadata.

---
This project has received funding from **Horizon Europe** research and innovation programme under grant agreement **no. 101135784**.

<p align="left">
  <!-- ---------- ARISE logo ---------- -->
  <!-- Light mode -->
  <img src="repo_images/EN_FundedbytheEU_RGB_POS.png#gh-light-mode-only" alt="EU Funding for light mode" height="100"/>

  <!-- Dark mode -->
  <img src="repo_images/EN_FundedbytheEU_RGB_NEG.png#gh-dark-mode-only" alt="EU Funding for dark mode" height="100"/>

</p>