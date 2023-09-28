# DIY-Arduino-Incubator
This is a repository for the ongoing "DIY arduino incubator from scratch" series in kaka_tv's channel. Here i will be sharing the hardware files (schematics, pcbs component list and their explanations) as well as source files for the program. 

The main goal of this project is designing a universal incubator that will run on an Arduino nano with as few pre made modules as possible. writing code that will cater to different people in different cities of the world. an easy and repairable incubator that can be set up and run even by a kid. all the while serving as a challenge and teaching us things about electronics and programming. 

This readme or wiki (later) will be in line with the youtube series.

## Episode 5:
I have already kick started designing the schematic for this. 
Schematic link in easyeda: https://easyeda.com/editor#id=|8790f6cc49c243429d678afd4571d37c
Video link: https://youtu.be/of25-rBlspA

### Plan for next video:
Start the software process:
1. the objectives of the incubator
2. difficulties in solving thsoe objectives
 
## Episode 6:
Just wanted to let the audience know which ide i was using and taught some basics about vscode and github. 
https://www.youtube.com/watch?v=H_leyX6zwIo


## Episode 7:
I wanted to first make the software and see if it ran alright, rather than build and test at the same time. it certainly took a lot of time if you take a look at the commit history even after working on this for over a year it still took me quite some time. I think it paid off. I am going to cover all the details about the code. 
unfortunately, I won't be able to go into as many basics as I wanted to due to the length and complexity of the code. but I will certainly leave links to the important stuff 
in the description of the video. 

Just merged the dev branch after testing almost all functions. now time to build on that and make it better, prettier and easier to understand. 
I will share my findings from when i was building this project in the video because there is just alot to cover and typing requires too much time. 
Also, i haven't really committed properly just because I kept trying different ways to make the code work for almost any situation so regular and correct 
commits were not an option. 


## Update July/07/2023

It's certainly been a while. having learned a lot of new things I'd like to integrate that into the incubator. so there's a change of plans. I will not be introducing Arduino into it. part of the reason is that I mainly want to turn this into a product instead of a youtube series. so I will certainly make videos about it but it won't be like a course. 

### rough plan
1. using esp32
2. supporting OTA updates
3. using mobile phones as the main interface
4. Store each user's data in a database like Firebase.
5. with fleet management software will control all the products and I can easily solve problems with any device.
6. having pre-defined sensors that will be shipping with the board instead of preparing it for any kind of sensor.
7. a one-page setting up instruction that will make it easy for the user to get started.
8. think about power-saving modes and making it developer friendly.
9. interface will be platform independent; web, android, ios. probably will base it on the web and serve it with Flutter.


### parts of the project 
1. Android app 
2. Hardware
3. microcontroller programming

* to make this a bit easier I'm going to use react native for app development. 
* should start working on the app even without hardware as I can set up a Firebase database FAST that will emulate the working of hardware. 
* for microcontroller programming, I'm going to use a jtag programmer (can be programmed with just USB) and will use idf for it. video  resources from learnesp32.com will be a huge help.


 --- UPDATE 9/28/2023 ---
### EXPECTED APP CAPABILLITIES 

#### features

1. login/signup screen
- login/signup via email/facebook/gmail 
- each user will have personal profile, details are not duplicated


2. dashboard screen 

#### pair (complex method but will guide about implementing it) 
- when clicked on pair should open the camera to scan a barcode
- access the captured barcode, intiate the pairing routine
- shows pairing status, if alive signal is updated constantly then currently paired

#### live info (gets data from firebase and shows it only)
- temp, humidity, water level, intensity of heater, rotation done/total rotations per day, next rotation scheduled for 

* hatch settings (gets and sets data to firebase) 
- hatch no, hatch status(on going or off), days left for hatch, hatch profile selected, hatch diary (get's detailed about success rate and any other notes the user might want to save, save pictures etc), initiate new hatch, cancel hatch, hatch alarms setup

#### current hatch settings 
- updates a flag in firebase that will override the current settings selected
and will implement settings made in this page
- temp, rotations per day, hatch period etc 

#### history 
- shows a graph of temperature over time, divided into daily graphs 
 as well as average graph until now 
-warnining history
- shows any power failure or any mis happenings, it's time, cause and solution 

- certain hatch history (stores all the information mentioned below will be saved under different hatches
should contain history of alarms, warning, incubator version and all the current settings from the user) 
- abillity to share that history as an excel file or word document. 

#### sensor info (get realtime update from firebase
- sensors status (OK/NA(not available)/FAULTY) such as temp, humidity, water sensor, door sensor

#### software info (get data from firebase) 
- version currently running, last OTA updated, available update,roll back last update


3. settings 
- profile settings(picture, bio, name, username )

#### hatch profile settings
- selects a profile like (chicken eggs or parrot eggs)changes default temp, humidity level and rotations

- alarm settings
- lets the user edit alarms such as
- alarm conditions, toggle preset alarms, alarm history(records and shows all the alarm over that hatch) 

4. help 
- hatching wiki (a book containing detail about hatching eggs) 
- questions (lets user search for questions and finds answer to that from book or internet) 
- contact support 
- complain
- any other helping material 


5. about us 
- details about the company



#### Basic features 

1. signup/login screen with email password. authenticates with firebase

- Start hatch  (button)
- pause hatch  (button)
- abort hatch  (button)


2. dashboard with text boxes that will show current value fetched from firebase 
the text boxes are following 
- temperature in degree celcius. (0-100 float)
- humidity in percentage (whole number ranging from 0-100)
- water reservoir level
- rotation done/total rotation (whole number) 
- days/total days(usually 21) (whole number)
- errors (horizontal box with images of sensors such as temp and humidity with tick mark or cross) 


3. settings
has editables text boxes that updates corresponding firebase value
note that all these text boxes will fetch last saved value from firebase
which will then be edited if desired. 
 
- setpoint temperature
- setpoint humidity
- setpoint rotations per day 
- incubation period (limit 1-60 days) 
- stop rotation alarm (out of 21 choose a day. limit is total hatch day) 
- setpoint temperature limit (user is notified if temp exceeds this) 
- setpoint humidity limit 
- alarm, user choosing date and time and a note. user is notified on the app
- alarm will have an action attached to it that can be turned on if needed
- actions are following; stop turning, change temp, change humidity,
- set PID values 
- motor turn time (time taken to turn the eggs enough to tilt them) 
- Reset all settings 


4. history 
shows graph of 24 hours of data. after every 24 hour data is rewritten.
- temperature history graph
- humidity history graph


5. notifications
all these notification should have the abillity to notify even if the 
screen is turn off or the app is not currently being used. 

- over/low temperature (when current temperature is above/below preset value for 2 minutes, hatch has to be in progress for this) 
- over/low humidity (when current humidity is above/below preset value)
- incubator turned off (will poll for an alive signal from firebase. if
that signal is not updated it will consider the incubator to be off) 
- water level low 
- door open 





