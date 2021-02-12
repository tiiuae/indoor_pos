#!/usr/bin/python3

import sys
import json
import requests
import argparse
from datetime import datetime

hostname = "https://globalmagnet.amentum.space/wmm/magnetic_field"

parser = argparse.ArgumentParser(description='Get magnetic declination')
parser.add_argument('--lat', type=float, action='store', required=True,
                    help='Latitude in decrees [ -90.0(S) .. 90.0(N) ]')
parser.add_argument('--lon', type=float, action='store', required=True,
                    help='Longitude in decrees [ -180(W) .. 180.0(E) ]')
parser.add_argument('--alt', type=float, action='store', default=0.0,
                    help='Altitude from MSL in meters [ 0.0 .. 600000.0 ]')

args = parser.parse_args()

date = datetime.now()
ydec = date.year + float(date.strftime("%j"))/365

payload = dict(
    altitude = args.alt / 1000, # [km]
    longitude = args.lon, # [deg]
    latitude = args.lat,
    year = ydec # decimal year, half-way through 2020
)

try:
  response = requests.get(hostname, params=payload)
  # extract JSON payload of response as Python dictionary
  json_payload = response.json()
  # raise an Exception if we encoutnered any HTTP error codes like 404 
  response.raise_for_status()
except requests.exceptions.ConnectionError as e: 
  # handle any typo errors in url or endpoint, or just patchy internet connection
  print(e)
except requests.exceptions.HTTPError as e:  
  # handle HTTP error codes in the response
  print(e, json_payload['error'])
except requests.exceptions.RequestException as e:  
  # general error handling
  print(e, json_payload['error'])
else:
  json_payload = response.json()
  #print(json.dumps(json_payload, indent=4, sort_keys=True))
  print(json_payload["declination"]["value"])
