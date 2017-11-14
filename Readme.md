To run scraping:

1. Launch PR2 simulator:

   `roslaunch giskard_pr2 interactive_markers.launch sim:=true`
  
2. Launch Gazebo:

   `roslaunch skill_transfer scraping_world_pr2.launch`
  
3. Launch the task:

   `roslaunch skill_transfer scraping_task.launch sim:=false`
  
   `sim:=true` would mean we don't want to simulate PR2 and use flying grippers directly
