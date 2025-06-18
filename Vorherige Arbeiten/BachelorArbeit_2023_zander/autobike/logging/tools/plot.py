import subprocess
import os
import shutil
import csv
import sys

gnuplot_bno_commands = '''cd "../plots"
set datafile separator ","
set key box
set key title "BNO Thread Data {}"
set yrange [-16:16]
set ytics nomirror
set y2tics
set y2range [-250:250]
# set xrange [0:11] # this is commented so it finds the range automatically
set xrange [{}:{}]
set xlabel "time in s"
set xtics 0,1
set ylabel "angle in degrees"
set y2label "encoder setpoint value"
set term png size {},1080
set output "{}"
plot "{}" \
skip 1 using ($1/1000000):3 title "angle" axis x1y1  with linespoints, \
"" skip 1 using ($1/1000000):11 title "angle setpoint" axis x1y1  with linespoints, \
"" skip 1 using ($1/1000000):6 title "p value" axis x1y2  with linespoints, \
"" skip 1 using ($1/1000000):8 title "d value" axis x1y2  with linespoints'''
#"" skip 1 using ($1/1000000):7 title "i value" axis x1y2  with linespoints''' #, \
#"" skip 1 using ($1/1000000):9 title "error" axis x1y1  with linespoints'''
#"" skip 1 using ($1/1000000):5 title "steering value" axis x1y2  with linespoints, \
#"" skip 1 using ($1/1000000):8 title "d value" axis x1y2  with linespoints, \

gnuplot_bno_commands_2 = '''cd "../plots"
set datafile separator ","
set key box
set key title "BNO Thread Data {}"
set yrange [-61:62]
set ytics nomirror
set y2tics
set y2range [-3:3]
# set xrange [0:11] # this is commented so it finds the range automatically
set xrange [{}:{}]
set xlabel "time in s"
set xtics 0,1
set ylabel "angle in degrees"
set y2label "encoder setpoint value"
set term png size {},1080
set output "{}"
plot "{}" \
skip 1 using ($1/1000000):10 title "steering offset" axis x1y1  with linespoints, \
"" skip 1 using ($1/1000000):11 title "angle offset" axis x1y1  with linespoints, \
"" skip 1 using ($1/1000000):12 title "1 left 2 right" axis x1y2  with linespoints, \
"" skip 1 using ($1/1000000):13 title "1 forward" axis x1y2  with linespoints'''

gnuplot_hnm_commands = '''cd "../plots"
set datafile separator ","
set key box
set key title "HNM Thread Data {}"
set yrange [0:7]
set ytics nomirror
set y2tics
set y2range [0:60000]
# set xrange [0:11] # this is commented so it finds the range automatically
set xrange [{}:{}]
set xlabel "time in s"
set xtics 0,1
set ylabel "current speed"
set y2label "hnm pwm value"
#set term png size 2048,1080
set term png size {},1080
set output "{}"
plot "{}" \
skip 1 using ($1/1000000):2 title "speed" axis x1y1  with linespoints, \
"" skip 1 using ($1/1000000):3 title "speed setpoint" axis x1y1  with linespoints, \
"" skip 1 using ($1/1000000):4 title "pid value" axis x1y2  with linespoints, \
"" skip 1 using ($1/1000000):6 title "i value" axis x1y2  with linespoints,\
#"" skip 1 using ($1/1000000):5 title "p value" axis x1y2  with linespoints, \
#"" skip 1 using ($1/1000000):7 title "d value" axis x1y2  with linespoints, \
#"" skip 1 using ($1/1000000):8 title "error" axis x1y1  with linespoints'''

gnuplot_md49_commands = '''cd "../plots"
set datafile separator ","
set key box
set key title "MD49 Thread Data {}"
set yrange [-160:160]
set ytics nomirror
#set y2tics
#set y2range [-100000:100000]
# set xrange [0:11] # this is commented so it finds the range automatically
set xrange [{}:{}]
set xlabel "time in s"
set xtics 0,1
set ylabel "encoder value"
set y2label "pwm steering value"
set term png size {},1080
set output "{}"
plot "{}" \
skip 1 using ($1/1000000):3 title "encoder setpoint" axis x1y1 with linespoints, \
"" skip 1 using ($1/1000000):2 title "encoder value" axis x1y1 with linespoints, \
# "" skip 1 using ($1/1000000):4 title "steering value" axis x1y2 with linespoints'''



print("visualization started...")

for filename in os.listdir("../logging_files"):
    
    # Setting start and stop values for the x axis
    begin_time = ""
    end_time = ""
    if len(sys.argv) >= 3:
        begin_time = sys.argv[1]
        end_time = sys.argv[2]
    
    # Setting the horizontal pixelcount either manually or automatically
    pixelcount = 2048
    if len(sys.argv) == 4:
        pixelcount = int(sys.argv[3])
    else:
        runnint_time = 100
        filepath = "../logging_files/" + filename
        with open(filepath, "r") as file:
            csv_reader = csv.reader(file)
            try:
                running_time = int(list(csv_reader)[-2][0])
            except:
                print("file empty...")
                continue
        pixelcount = max((int) (running_time*4e-5), 2048)
    
    
    if "bno" in filename:
        temp_gnuplot = gnuplot_bno_commands.format(filename[:20], begin_time, end_time, pixelcount, filename[:20] + "_bno_plot.png", "../logging_files/" + filename)
        # temp_gnuplot += "\n" + gnuplot_bno_commands_2.format(filename[:20], begin_time, end_time, pixelcount, filename[:20] + "_bno_plot2.png", "../logging_files/" + filename)
    elif "md49" in filename:
        temp_gnuplot = gnuplot_md49_commands.format(filename[:20], begin_time, end_time, pixelcount, filename[:20] + "_md49_plot.png", "../logging_files/" + filename)
    
    elif "hnm" in filename:
        temp_gnuplot = gnuplot_hnm_commands.format(filename[:20], begin_time, end_time, pixelcount, filename[:20] + "_hnm_plot.png", "../logging_files/" + filename)
    
    
    with open("temporary_gnuplot_commands.gp", "w") as file:
        file.write(temp_gnuplot)
        
    print("file " + filename + " visualized")
    subprocess.run(["gnuplot", "-c", "temporary_gnuplot_commands.gp"], check=True)

    os.remove("temporary_gnuplot_commands.gp")
    shutil.move("../logging_files/" + filename, "../logging_files_visualized")


print("All files visualized!")
exit()
