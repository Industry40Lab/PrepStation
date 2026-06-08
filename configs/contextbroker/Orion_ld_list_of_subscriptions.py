import requests
import json

url = "http://localhost:1026/ngsi-ld/v1/subscriptions/"

payload = {}
headers = {
  'Content-Type': 'application/json'
}

response = requests.request("GET", url, headers=headers, data=payload)

print(response.text)
