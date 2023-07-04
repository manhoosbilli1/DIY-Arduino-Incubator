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
