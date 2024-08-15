# Machine-Learning-Based-Electronic-Scarerow-for-Protecting-Agricultural-yields-Rice-Farm-

# Electronic Scarecrow with Bird Detection and Deterrence

This project implements an electronic scarecrow using a Raspberry Pi 4B to detect and deter birds from rice farm. The system utilizes a YOLOv8 model for bird detection, camera to detect the presence of bird, and activate deterrence mechanism.

## Components Used
- **Raspberry Pi 4B**
- **YOLOv8 Bird Detection Model**
- **DFRobot MP3 Player Module**
- **BTS 7960 Motor Driver**
- **PiCamera**

## Repository Link
You can find the source code and necessary files in the following GitHub repository: [GitHub Repository Link]

## Prerequisites
Before you begin, make sure you have the following installed on your Raspberry Pi:
- Python 3
- PiCamera library
- TensorFlow or PyTorch (depending on which backend you are using with YOLOv8)
- Required Python packages (see below)
- Ultralytics library

#You may create virtual environment to work in

## Setup Instructions

### 1. Clone the Repository
First, clone the GitHub repository to your Raspberry Pi:
```bash
git clone [GitHub Repository Link]
cd Electronic-Scarecrow






2. Install Python Dependencies
#Install the required Python libraries by using the below command

pip install -r requirements.txt

3. Configure Hardware
#Connect the DFRobot MP3 Player:
#Connect the MP3 Player module to the Raspberry Pi according to the DFRobot MP3 Player Guide.
#Make sure the MP3 files you want to play as bird deterrents are stored on a microSD card inserted into the MP3 player.
#Connect the BTS 7960 Motor Driver:
#Wire the BTS 7960 motor driver to control the 775DC motor which was conveterted to a vibrator motor 
#Use the GPIO pins on the Raspberry Pi to connect to the motor driver as per the motor driver’s datasheet.
#Attach the PiCamera:
#Connect the PiCamera to the dedicated camera port on the Raspberry Pi.
#Enable the camera interface in Raspberry Pi’s configuration:

#command
sudo raspi-config

Navigate to "Interfacing Options" > "Camera" and enable it.


4. Model Setup
Download the YOLOv8 pre-trained weights or use your custom-trained model. Place the weights file in the model/ directory:

Running the System
Once everything is set up, you can start the bird detection and deterrence system:
    
