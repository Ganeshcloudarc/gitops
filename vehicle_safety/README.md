# Vehicle safety

# About

- Vehicle Safety Package make sure the vehicle wont do anything abnormal incase of certain failure situations.
- Working issue - #25

# Current Status

-  [x] GPS failsafe
-  [x] cte failsafe
-  [x] emergency stop
-  [x] Geofence
-  [x] heading failsafe
-  [x] Add battery status check before starting patrol. if batt < 40, don't start vehicle
-  [x] Dynamic CTE based on turnings or straight line.
-  [x] Detect steering stuck and stop the vehicle - 3
-  [ ] Sevcon errors fail safe.
-  [ ] Launch patrol always
-  [ ] object detection
-  [ ] Keep Alive to BCU
-  [ ] sensor update and fault monitoring
-  [ ] speed failsafe
-  [ ] Error Codes
-  [ ] Nodes monitoring
-  [ ] ensuring proper kill and exit from the launch files
-  [ ] ros without network
-  [ ] NRU memory handling

| Function  | Description | Trigger Action |
|---|---| --- |
| GPS FailSafe  | When RTK fix is poor (< 5) | STOP |
| cte failsafe |  When Cross Stack Error > 8m | STOP |
| emergency stop  |  WARN when Emergency STOP  | STOP |
| Geofence | Check whether vehicle is under safe area, if not Trigger  | STOP
| heading failsafe  | When Heading error > 10 degrees  | STOP |

# Parameters

- [Params](params/vehicle_safety_params.yaml)

```
vehicle_safety:
  CTE_THR : 8 # cte threshold in meters
  HEAD_THR : 10 # Heading Threshold in degrees(+/-)
  GPS_FIX_THR : 10.0 #GPS FIX Threshold time in seconds
  use_geo_fence : True
  geo_fence_coordinates: [
    [13.584927310648094,79.96143747121096],
    [13.584685821553478,79.96227599680424],
    [13.584415001507033,79.96305719017982],
    [13.585372158336437,79.96300991624594],
    [13.585495672674751,79.96209863573313],
    [13.585544231064546,79.96139656752348],
    [13.584927310648094,79.96143747121096]
]
```

- If use geo_fence, make sure use_geo_fence: True.
- Update GeoFence coordinates in the param file.
    - Generate Ploygon in the website, http://apps.headwallphotonics.com/
    - make sure the last and first coordinates are same to close polygon