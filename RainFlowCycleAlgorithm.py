
import logging
import logging.config

# Configure logging from the logging.conf file
logging.config.fileConfig('logging.conf')

import matplotlib.pyplot as plt
import numpy as np
plt.set_loglevel(level='warning')


class RFCA:
    def __init__(self, data):
        self._l = logging.getLogger("RFCA")
        self._l.info("Initializing RFCA.")
        self.step = 0   
        self.flows = []
        self.active_flows = []
        self.Atmp = [0,0]
        self.cycles = []
        self.data = data
    
    def get_cycles(self):
        self._l.info("Getting cycles.")
        self.cycles = []
        for flow in self.flows[:-1]:
            cycle_range = round(abs(flow[3] - flow[2]))
            tmp_list = [cycle[0] for cycle in self.cycles]
            if cycle_range in tmp_list:
                cycle_idx = tmp_list.index(cycle_range)
                self.cycles[cycle_idx][1] += 0.5
                #self._l.info(f"Cycle range: {cycle_range}, Cycle index: {cycle_idx}")
            elif cycle_range > 0:
                self.cycles.append([cycle_range, 0.5])

            self.cycles = sorted(self.cycles)
            #self._l.info(f"Cycles: {self.cycles}")
        self._l.info(f"Data: {self.data}")
        self._l.info(f"Data: {self.flows}")
        self._l.info(f"Data: {self.cycles}")
        return self.cycles

    def get_flows(self):
        return self.flows
    
    def get_active_flows(self):
        return self.active_flows
    
    def update_if_peak(self, new_data):
        is_peak = True
        if self.Atmp[0] <= self.Atmp[1] and self.Atmp[0] < new_data:
            self.data.append(round(self.Atmp[0]))
            self.counter_step(self.data[-1])
        elif self.Atmp[0] >= self.Atmp[1] and self.Atmp[0] > new_data:
            self.data.append(round(self.Atmp[0]))
            self.counter_step(self.data[-1])
        else:
            is_peak = False
        self.Atmp = [new_data, self.Atmp[0]]
        if is_peak:
            self._l.info(f"New peak found: {self.data[-1]}")
        return is_peak

    def counter_step(self, new_data):
        self.step = self.step + 1
        self._l.info(f"Running Counter Step: {self.step}")
        self._l.info(f"Peaks: {self.data}")
        current_max = new_data
        tmp_active_flows = self.active_flows.copy()
        terminated_flows = []
        first = True          
        for idx, flows in enumerate(tmp_active_flows):
            #self._l.info(f"Current is first: {first}")  
            flows[1] = self.step

            if flows[0] == self.step-1:
                flows[3] = (current_max)
                if not first:
                    #self._l.info(f"Flow {flows[0]} terminated.")
                    terminated_flows.append(idx)
                if new_data > flows[2]:
                    flows.append(+1)
                else:
                    flows.append(-1)
                self.flows[flows[0]-1] = flows
                first = False
            else:
                match flows[4]:
                    case -1:
                        if new_data > flows[2]:
                            #self._l.info(f"Flow {flows[0]} terminated.")
                            terminated_flows.append(idx)
                        elif new_data <= flows[3]:
                            #self._l.info("New data is less than the second peak, peak exteded.")
                            tmp_max = flows[3]
                            self.active_flows[idx][3] = current_max
                            if not first:
                                #self._l.info(f"Flow {flows[0]} terminated.")
                                terminated_flows.append(idx)
                            current_max = tmp_max
                            self.flows[flows[0]-1] = flows
                            first = False

                        else:
                            self._l.info("New data is between the two peaks.")
                    case 1:
                        if new_data < flows[2]:
                            #self._l.info(f"Flow {flows[0]} terminated.")
                            terminated_flows.append(idx)
                        elif new_data >= flows[3]:
                            #self._l.info("New data is less than the second peak, peak exteded.")
                            tmp_max = flows[3]
                            self.active_flows[idx][3] = current_max
                            if not first:
                                #self._l.info(f"Flow {flows[0]} terminated.")
                                terminated_flows.append(idx)
                            current_max = tmp_max
                            self.flows[flows[0]-1] = flows
                            first = False
                        else:
                            self._l.info("New data is between the two peaks.")
            #self._l.info(f"Processing flow number: {flows}")
        self.active_flows.append([self.step, self.step, new_data, new_data])
        self.flows.append([self.step, self.step, new_data, new_data])

        for idx in sorted(terminated_flows, reverse=True):
            #self._l.info(f"Removing flow {idx} from active flows.")
            self.active_flows.pop(idx)

        #self._l.info(f"Active flows: {self.active_flows}")
           
            
            

    def rerun_counter(self):
        self._l.info("Rerunning Counter Steps.")
        self.step = 0
        self.active_flows = []
        for i in range(len(self.data)):
            self.counter_step(self.data[i])
        return self.flows
    
    def get_flow_coordinates(self, flow):
        data = self.data
        x = [flow[0]-1]
        y = [flow[2]]
        for idx,j in enumerate(range(flow[0], flow[1])):
            if flow[4] == -1 and data[j] < y[idx]: 
                y.append(y[-1])   
                xdiff = abs(1/(data[j] - data[j-1])*(data[j] - y[-1]))
                x.append(j-xdiff)

                #Add Peak
                y.append(max(flow[3],min(y[-1],data[j])))
                xdiff = abs(1/(data[j] - data[j-1])*(data[j] - y[-1]))
                x.append(j-xdiff)
                #x.append(j)
            elif flow[4] == 1 and data[j] > y[idx]: 
                y.append(y[-1])   
                xdiff = abs(1/(data[j] - data[j-1])*(data[j] - y[-1]))
                x.append(j-xdiff)

                #Add Peak
                y.append(min(flow[3],max(y[-1],data[j])))
                xdiff = abs(1/(data[j] - data[j-1])*(data[j] - y[-1]))
                x.append(j-xdiff)
                #x.append(j)
            else:
                y.append(y[-1])   
                x.append(j)
        #self._l.info(f"Flow coordinates: {x}, {y}")
        return x, y
                
            
    def plot_flows(self):
        self._l.info("Drawing results.")
        # Placeholder for drawing logic
        # This should include the logic to visualize the results

        plt.figure()
        plt.plot(self.data, '-*', label='Peak', color = 'black', linewidth=1, alpha=0.5)
        for flows in self.flows:
            x,y = self.get_flow_coordinates(flows)
            plt.plot(x,y,':', linewidth = 3)
        plt.xlabel('Time Step')
        plt.ylabel('Horizontal Load [N]')
        plt.title('Horizontal Load Over Time')
        plt.legend()
        plt.grid()
        plt.show()

    def plot_cycles(self):
        self._l.info("Drawing cycles.")
       
        fig2, ax = plt.subplots()   

        # Example data
        ranges = [cycle[0] for cycle in self.cycles]
        y_pos = ranges
        cycles = [cycle[1] for cycle in self.cycles]

        ax.barh(y_pos, cycles, align='center')
        ax.set_yticks(y_pos, labels=ranges)
        ax.set_xlabel('Number of Cycles')
        ax.set_title('Rainflow Cycles')

        plt.show()




    
