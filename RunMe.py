"""
Reactive Pure Pursuit Control Based AGV Path Planner 
@author: YaZhou Wang
"""
import matplotlib.pyplot as plt
import caculate_path_length
import path_planner
def plot_result(all_calculation_time,all_path_length):
    count_failed = 0
    success_rate = 0
    average_compute_time = 0
    average_path_length = 0
    whole_time = 0
    whole_lenth = 0
    for i in range(len(all_calculation_time)):
        if all_calculation_time[i]=="plan failed":
            count_failed+=1
        elif all_calculation_time[i]=="failed":
            count_failed+=1
        elif all_calculation_time[i] > 10:
            count_failed+=1
        else:
            whole_time += all_calculation_time[i]
            whole_lenth += all_path_length[i]
    success_rate = 1 - (count_failed)/len(all_calculation_time)
    average_compute_time = whole_time/(len(all_calculation_time) - count_failed)
    average_path_length = whole_lenth/(len(all_calculation_time) - count_failed)
    bar = plt.bar(x = [1,2,3],height = [success_rate*100,average_path_length,average_compute_time],width = 0.35,label = ['%','m','s'],
                  color = 'orange',tick_label = ['SuccessRate','AveragePathLength','AverageComputeTime'],linewidth= 3)
    plt.bar_label(bar)
    plt.legend()
    plt.show()
def main():
    print('path planning started!')
    all_calculation_time=[]
    all_path_length=[]
    add_number=0
    start_number=1
    end_number=100
    for j in range(10):
        start_number = start_number + j*100
        end_number = end_number + j*100
        add_number = add_number + j*100        
        path = "benchmark_cases/obstacle_map_"+str(start_number)+"_"+str(end_number)+".txt"             
        file = open(path,'r',encoding= 'UTF-8')       
        a_file=file.readlines()
        counter=-1
        for i in range(0,len(a_file)-1,3):
            counter+=1
            ox=[eval(a_file[i].strip())][0]
            oy=[eval(a_file[i+1].strip())][0]
            obs=[eval(a_file[i+2].strip())][0]
            result_time,result_route=path_planner.pco_planning(ox,oy,obs,counter+add_number) 
            if type(result_time)!=type(1.0):
                all_calculation_time.append(result_time)
                all_path_length.append(result_route)
            else:
                all_calculation_time.append(result_time)
                s=caculate_path_length.add_path(result_route.x,result_route.y)
                all_path_length.append(s)
                 
    plot_result(all_calculation_time,all_path_length)
if __name__ == "__main__":
    main()