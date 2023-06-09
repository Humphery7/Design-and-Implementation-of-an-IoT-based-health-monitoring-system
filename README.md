# Design-and-Implementation-of-an-IoT-based-health-monitoring-system
This project is aimed at improving medical responses to high-risk individuals

**Background of Study**
Health is a fundamental sector in every sphere of the world as it is the only known mechanism that can cheat death. However, it is saddening that the 21st century Nigeria struggles with high mortality rate even though she claims to be the giant of Africa. As a world, we prove to have moved so much, however, we are still plagued by some medical conditions that are seemingly beyond the scope of our knowledge of Biology, however, possessing information about a medical condition, or a trend may shed light to the medical practitioner’s blind spot.  Some of the key bottlenecks attesting to the Nigerian inept health care system are:
1.	Nigeria has a low doctor to patient ratio, about 3.8 doctors per 10,000 citizens (globalcitizen.org)).
2.	Data collection and record keeping in the health sector is poor in the nation, and most of the record keeping is done by UNICEF, international nonprofits and the WHO (globalcitizen.org)).
3.	According to ( Health Guide NG), full medical check-up in Nigeria falls in the range of N50,000-N150,000, which is not affordable as the minimum wage in Nigeria is N30,000.


**Problem Statement**
Wearables give people lightweight and immediate access to messages, notifications and other digital data while on-the-go. While already powerful as standalone devices, the capabilities of wearables increase significantly when used in tandem with other devices that people carry, such as their phones or tablets, which allows for novel cross-device interaction techniques and although not to be used for medical diagnosis, can provide useful information on the health status of an individual. 

However, a number of existing wearables devices that give information on health do not take into consideration the knowledge of the user on the health status given especially when these values indicate possible health issues and when they indicate normal functioning of the person. 

Finally, the capabilities of the wearables can be extended to much more than just health indications, one of such importance, is the capability to detect fall of an individual and send a message across to trusted persons of such individuals. This extra capability will be found to be very purposeful in cases where the user is in a delicate health position, such as (Stroke patients, Aged, Pregnant persons, high risk persons etc.) as a fall occurring in this subset of individuals could require immediate response. In summary, the gospel this project preaches is that Wearables can be extended to do much more than display time, make phone-calls, play games etc., they can be involved in providing insight about medical conditions that seem difficult to combat on the go. 
1.2	Aims and Objectives
1.	To provide a means of health check for groups of persons unable to get medical check-up, through highly accurate supervised predictions from data obtained.
2.	To improve early diagnosis, so as to prevent aggravated illness or sudden death, by a system that indicates whenever it measures values that show there may be a health issue.
3.	To indicate and ensure faster response when a “high-risk individual” falls.


**Scope 
Limitations**
The choice of the Arduino Nano development board due to its portable size resulted into inability to use readymade libraries for most of the sensors used for this project because the Arduino Nano has just 32KB FLASH  memory and 2KB SRAM, hence, memory optimization was highly prioritized. This memory constraint also inhibited us from exploring some of our dearest functionalities like collecting data from the pulse sensor and temperature sensor and using machine learning to help the medical practitioner to give a credible report of the health status of their patients.
 The IOT interface chosen was ThingSpeak, however, we wanted to move beyond the world of data subscription and explore cases where there are poor internet connections, the SIM8001 module would have been the most perfect choice for this option, however, it is cost intensive and the memory constraint on the Arduino Nano board militates against this reality. The Pulsesensor used for this project is not the state of art Pulsesensor as it is highly prone to noise and may output questionable values sometimes. The simulation of the project was also impossible due to the presence of the Wi-fi module. 
 
 
**Project Concept**
This Project will be designed as a Wearable Device System; however, this will be different from other Wearables as the concept of Machine Learning will be integrated into it. This Project will consist of 3sensors (could be expanded, left as 3 for financial and simplicity reasons), temperature sensor, Pulsesensor module , and a MPU-6050 3-axis Accelerometer(this specific sensor is required for fall detection), fall detection becomes an important aspect when the human person is a high risk individual; whenever a fall is detected in these group of persons, a message is sent to their trustees that such has happened, this is important as such person’s life could be saved.
