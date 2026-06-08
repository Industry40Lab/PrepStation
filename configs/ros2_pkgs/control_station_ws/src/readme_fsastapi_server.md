1 - run the server ( the FAST library install on the virtual environment )
```bash
    source ~/fastapi-env/bin/activate
    uvicorn run_server:app --host 0.0.0.0 --port 5000
```
 


2 - upload the image to the server
```bash
curl -F "file=@Scr.png" http://localhost:5000/upload/
```
then you will get the corresponding url:
```bash
 {"image_url":"http://localhost:5000/images/6fe308723efc48279bd7884ecc7335eb.png"}
```

3 - find the ip of the server in the network ( do not use polimi network)

 ```bash
  ip a
 ```
4 - the ultimate url will be sth like this
```bash 
    http://10.169.116.107:5000/images/6fe308723efc48279bd7884ecc7335eb.png
```

5 - run the vulcanexus-jazzy docker 
```bash
sudo docker run -it --rm --name=ros2 \
--net=host --ipc=host --privileged -e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/mostafa/workspaces/projects/Arise/usecase1/secondyear_development/control_stattion_ws/src:/my_ws/src \
eprosima/vulcanexus:jazzy-desktop \
bash

```
6- if you want to open a second terminal 
```bash
sudo docker exec -it ros2 bash
source /opt/vulcanexus/jazzy/setup.bash
source /my_ws/install/setup.bash
```
7 - The synchronization automatically occures between the directory on host and the corresponding files on the container if you added option -v in the docker run



9 - run the image publiser 

```bash
ros2 run defective_pcb_detector defective_pcb_publisher
```

10 - upload the images to server [on local host]
```bash
ros2 run defective_pcb_detector defective_pcb_to_server http://0.0.0.0:5000
```
pip install --break-system-packages requests

11  - sudo docker compose -f docker-composev2.yml up -d


sudo docker stop $(sudo docker ps -q)

sudo docker ps

sudo docker logs -f orion_dds_test-orion-1
sudo docker stop $(sudo docker ps -q | tail -n +2)


12 - create the subscription to the orion_ld
```bash
curl --location 'http://localhost:1026/ngsi-ld/v1/subscriptions/' \
--header 'Content-Type: application/json' \
--data '{
  "description": "Monitor changes to the image URL of PCB",
  "type": "Subscription",
  "entities": [
    {
      "type": "Robot",
      "id": "urn:ngsi-ld:pcb:1"
    }
  ],
  "watchedAttributes": ["mypcb"],
  "notification": {
    "attributes": ["mypcb"],
    "endpoint": {
      "uri": "http://localhost:8668/v2/notify",
      "accept": "application/json"
    }
  }
}'
```


13 - check subscription 
```bash
curl --location 'http://localhost:8668/v2/entities/urn:ngsi-ld:pcb:1/attrs/mypcb?lastN=3' \
--header 'Accept: application/json'
```
14 grafana query request

```bash
SELECT 
  mypcb['defected'] AS defected,
  mypcb['material_info'] AS material_info,
  mypcb['heatsink_number'] AS heatsink_size,
  mypcb['url'] AS image_url,
  mypcb['id'] AS pcb_id,
  mypcb['defect_loc_x'] AS x,
  mypcb['defect_loc_y'] AS y,
  mypcb['departured'] AS departured
FROM "doc"."etrobot"
WHERE mypcb['url'] IS NOT NULL
ORDER BY time_index DESC
LIMIT 1;
```

then choose Business Text plugin, and copy this html code for rendering of json info
```bash
      <h2>Defective PCB Info</h2>
      <p><strong>ID:</strong> {{@root.pcb_id}}</p>
      <p><strong>Defected:</strong> {{@root.defected}}</p>
      <p><strong>Material:</strong> {{@root.material_info}}</p>
      <p><strong>Defect Location:</strong> ({{@root.x}}, {{@root.y}})</p>
      <p><strong>Departured:</strong> {{@root.departured}}</p>

      <figure style="text-align: center;">
        <img src="{{{ @root.image_url }}}" alt="Defective PCB" width="400" style="border: 1px solid #ccc;" />
        <figcaption style="font-size: 14px; color: #666; margin-top: 8px;">
          Defective PCB detected on {{@root.departured}}
        </figcaption>
      </figure>



      ```json
      {{{json @root}}}
      ```
```

sudo docker rm -f grafana
docker run -d \
  --name=grafana \
  -p 3000:3000 \
  -e "GF_INSTALL_PLUGINS=marcusolsson-dynamictext-panel" \
  grafana/grafana
