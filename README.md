# hapkit
Programs for the hapkit haptic simulation device.

Developed by Cassondra Brown, Zach Brong, and Greg Yeutter for a project in ECES 690: Introduction to Haptics at Drexel University, Spring 2015. Our blog is [here](https://hapkit.wordpress.com/). Special thanks to the [RE Touch Lab](http://re-touch-lab.com/) for assistance and insight in getting this project running successfully.

Code is loosely based on [hapkit sample code](http://hapkit.stanford.edu/files/HapkitLab4SolutionsForWebsite.ino) as well as work by [Cliff Bargar](https://github.com/cliffbar/Hapkit_HW2).

Code summary:
* HapkitBump.ino: Simulate a bump with adjustable width.
* HapkitValley.ino: Simulate a valley with adjustable width.
* HapkitHardSurface.ino: Simulate collision with/pressing against a hard surface with adjustable spring constant.

After building a [hapkit](http://hapkit.stanford.edu/) (2014 version), the [Arduino IDE](http://www.arduino.cc/en/Main/Software) is installed. Each sketch (.ino file) is loaded into the IDE separately and uploaded to the hapkit control board. Only one simulation can be run at a time.

 After the sketch is uploaded, the serial monitor is opened from the IDE (Tools > Serial Monitor or ctrl-shift-m on Windows / cmd-shift-m on Mac). From the entry box, the characters 'A', 'B', or 'C' (without quotes, capitalized) can be entered to select the lowest, middle, or highest setting. A confirmation will be displayed on screen and the hapkit should adjust itself immediately.
