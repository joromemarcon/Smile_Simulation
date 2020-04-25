Example model -> road_solid_white

This tutorial assumes you already have scripts created for the visual aspect of your model. If you do not have desired visuals for your model included in file://media/materials/scripts/smile.material, you will not be able to add appropriate info in following step:
		    <script>
		      <uri>file://media/materials/scripts/smile.material</uri>
		      <name>Smile/Solid_Lane</name>
		    </script>

STEPS TO CREATE A ROAD:

Step 1: Create a folder with model name 
	ie: road_solid_white


Step 2: Create two files within new folder: model.config and model.sdf


Step 3: Place following code within model.config file, replacing model-name, name, email, and description as necessary.

	<?xml version="1.0"?>

	<model>
	  <name>Ground Plane</name>
	  <version>1.0</version>
	  <sdf version="1.5">model.sdf</sdf>

	  <author>
	    <name>Nate Koenig</name>
	    <email>nate@osrfoundation.org</email>
	  </author>

	  <description>
	    A simple ground plane.
	  </description>
	</model>


Step 4: Place the following code within model.sdf, changing model-name and script-name accordingly.

	<?xml version="1.0" ?>
	<sdf version="1.5">	
	  <model name='road_solid_white'>
	    <static>true</static>
	    <link name='link'>
	      <collision name='collision'>
		<geometry>
		  <plane>
		    <size>4 8</size>
		  </plane>
		</geometry>
		<surface>
		  <friction>
		    <ode>
		      <mu>100</mu>
		      <mu2>50</mu2>
		    </ode>
		  </friction>
		</surface>
	      </collision>
	      <visual name='visual'>
		<geometry>
		  <plane>
		    <size>4 8</size>
		  </plane>
		</geometry>
		<material>
		  <script>
		    <uri>file://media/materials/scripts/smile.material</uri>
		    <name>Smile/Solid_Lane</name>
		  </script>
		</material>
	      </visual>
	    </link>
	  </model>
	</sdf>


Step 5: Insert the new model into your gazebo world. Start Gazebo, then on the left-hand side find the tab labeled 'Insert'. Click it, then click 'Add Path'. Add the path of your 'models' folder. 
