# HKU UAS obstacle avoidance

This obstacle avoidance script is used to plan the flight mission in the SUAS2021 Competition. This script use A* to look for the shortest path and will export the path to the mission.plan file, which will be upload to Mavlink using drone kit.



This script will take the mission file from QGroundControl, and will replace with 'Requests' library to get mission data from interoperability server in production.

## Usage

1. Git the current repository
   `git clone https://github.com/asd65241/HKU-UAS-obstacle-avoidance.git`

2. Go to the folder
   `cd HKU-UAS-obstacle-avoidance`

3. Create .env file
   `pipenv shell`

4. Install the required python library

   `pip -r requirements.txt` 

5. Rename the mission file to:
   `mission.plan`

6. Run the script
   `python run.py`

7. Open map.html once the script is finished

## Demo

<img src="https://i.ibb.co/nsw9w5D/Screenshot-2021-03-16-at-8-07-09-PM.png" alt="Screenshot-2021-03-16-at-8-07-09-PM" border="0">

