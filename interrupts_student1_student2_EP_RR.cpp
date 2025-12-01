/**
 * 
 * @file interrupts.cpp
 * @author Sasisekhar Govind
 * 
 * @author Teereza Allaham (101289630) and Sahil Todeti (101259541) 
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 * 
 */

#include<interrupts_student1_student2.hpp>

void FCFS(std::vector<PCB> &ready_queue) {
    std::sort(
                ready_queue.begin(),
                ready_queue.end(),
                []( const PCB &first, const PCB &second ){
                    return (first.arrival_time > second.arrival_time); 
                } 
            );
}

/**
 * Sort ready queue by priority (ascending)
 * If equal priority → stable FIFO (RR)
 */
void sort_EP_RR(std::vector<PCB> &ready) {
    std::stable_sort(
        ready.begin(),
        ready.end(),
        [](const PCB &a, const PCB &b) {
            return a.priority < b.priority; // lower value = higher priority
        }
    );
}

std::tuple<std::string> run_simulation(std::vector<PCB> list_processes) {

    std::vector<PCB> ready_queue;   //The ready queue of processes
    std::vector<PCB> wait_queue;    //The wait queue of processes
    std::vector<PCB> job_list;      //A list to keep track of all the processes. This is similar
                                    //to the "Process, Arrival time, Burst time" table that you
                                    //see in questions. You don't need to use it, I put it here
                                    //to make the code easier :).

    PCB running;
    unsigned int current_time = 0;
    unsigned int RR_TIME = 100; // Declaring Round Robin with a 100 ms timeout
    
    //Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;

    //make the output table (the header row)
    execution_status = print_exec_header();

    //Loop while till there are no ready or waiting processes.
    //This is the main reason I have job_list, you don't have to use it.
    while (!all_process_terminated(job_list) || job_list.empty()) {

        //Inside this loop, there are three things you must do:
        // 1) Populate the ready queue with processes as they arrive
        // 2) Manage the wait queue
        // 3) Schedule processes from the ready queue

        //Population of ready queue is given to you as an example.
        //Go through the list of proceeses
        for (auto &process : list_processes) {
            if (process.arrival_time == current_time) {

                PCB p = process;
                assign_memory(p);  // Assign memory to the process

                p.state = READY; // Set the process state to READY
                p.cpu_executed = 0;
                p.quantum_used = 0;
                p.wait_until = 0;

                // Add process to ready queue and job tracking list
                ready_queue.push_back(p); // Add the process to the ready queue
                job_list.push_back(p); // Add it to the list of processes
                
                // Log the state transition from NEW to READY
                execution_status += print_exec_status(current_time, p.PID, NEW, READY);
            }
        }

        ///////////////////////MANAGE WAIT QUEUE/////////////////////////
        //This mainly involves keeping track of how long a process must remain in the ready queue
        // 2) Manage the wait queue
        for (auto it = wait_queue.begin(); it != wait_queue.end();) {
            // Check if I/O operation has completed
            if (it->wait_until == current_time) {
                PCB p = *it; // Move process from wait queue back to ready queue
                p.state = READY;

                ready_queue.push_back(p);
                sync_queue(job_list, p);

                // Log the state transition from WAITING to READY
                execution_status += print_exec_status(current_time, p.PID, WAITING, READY);

                it = wait_queue.erase(it);

            } 
            // Move to next process if still waiting
            else { 
                ++it;
            }
        }

        // Sort ready queue after arrivals & I/O
        if (!ready_queue.empty())
            sort_EP_RR(ready_queue);

        //////////////////////////SCHEDULER//////////////////////////////
        // 3) Schedule processes from the ready queue
        if (running.PID != -1 && running.state == RUNNING && !ready_queue.empty()) {

            PCB top = ready_queue.front(); // highest priority

            // Check if the ready process has higher priority 
            if (top.priority < running.priority) {
                // Preempt RUNNING → READY
                unsigned int time = current_time;

                // Log the preemption: RUNNING → READY
                execution_status += print_exec_status(time, running.PID, RUNNING, READY);

                // Move currently running process back to ready queue
                running.state = READY;  // Set the process state to READY
                running.quantum_used = 0;
                ready_queue.push_back(running); // Add the process to the ready queue
                sync_queue(job_list, running);

                idle_CPU(running); // Mark CPU as idle temporarily

                sort_EP_RR(ready_queue); // Re-sort ready queue to maintain priority order

                // Dispatch new one
                running = ready_queue.front();
                ready_queue.erase(ready_queue.begin());

                running.state = RUNNING;
                running.start_time = time;
                running.quantum_used = 0;

                sync_queue(job_list, running);

                // Log the state transition: READY → RUNNING
                execution_status += print_exec_status(time, running.PID, READY, RUNNING);
            }
        }

        // If CPU is idle and there are processes ready to run, dispatch one
        if (running.PID == -1 && !ready_queue.empty()) {

            // Take the highest priority process from the front of ready queue
            running = ready_queue.front();
            ready_queue.erase(ready_queue.begin());

            // Initialize the newly dispatched process
            running.state = RUNNING;
            running.start_time = current_time;
            running.quantum_used = 0;

            sync_queue(job_list, running);

            // Log the state transition: READY → RUNNING
            execution_status += print_exec_status(current_time, running.PID, READY, RUNNING);
        }

        // Execute the currently running process 
        if (running.PID != -1 && running.state == RUNNING) {

            // Decrement remaining time and increment execution counters
            running.remaining_time--;
            running.cpu_executed++;
            running.quantum_used++;

            unsigned int next_time = current_time + 1;

            // Check if process has completed all of its execution
            if (running.remaining_time == 0) {

                // Log the state transition: RUNNING → TERMINATED
                execution_status += print_exec_status(next_time, running.PID, RUNNING, TERMINATED);

                terminate_process(running, job_list);
                idle_CPU(running);

                // If there are ready processes, dispatch the highest priority one
                if (!ready_queue.empty()) {
                    sort_EP_RR(ready_queue); // Ensure ready queue is sorted by priority

                    running = ready_queue.front();
                    ready_queue.erase(ready_queue.begin());

                    running.state = RUNNING;
                    running.quantum_used = 0;
                    running.start_time = next_time;

                    sync_queue(job_list, running);

                    // Log the state transition: READY → RUNNING
                    execution_status += print_exec_status(next_time, running.PID, READY, RUNNING);
                }

                current_time = next_time;
                continue; // Skip to next iteration
            }

            // Check if process needs to perform I/O operation
            if (running.io_freq > 0 && running.cpu_executed == running.io_freq) {
                
                // Log the state transition: RUNNING → WAITING
                execution_status += print_exec_status(next_time, running.PID, RUNNING, WAITING);

                // Prepare process for I/O wait
                PCB w = running;
                w.state = WAITING;
                w.cpu_executed = 0;
                w.quantum_used = 0;
                w.wait_until = next_time + w.io_duration;

                // Move process to wait queue
                wait_queue.push_back(w);
                sync_queue(job_list, w);

                idle_CPU(running);

                // If there are ready processes, dispatch the highest priority one
                if (!ready_queue.empty()) {
                    sort_EP_RR(ready_queue); // Ensure ready queue is sorted by priority

                    running = ready_queue.front();
                    ready_queue.erase(ready_queue.begin());

                    running.state = RUNNING;
                    running.quantum_used = 0;
                    running.start_time = next_time;

                    sync_queue(job_list, running);

                    // Log the state transition: READY → RUNNING
                    execution_status += print_exec_status(next_time, running.PID, READY, RUNNING);
                }

                current_time = next_time;
                continue;
            }

            // Check if process has used up its time quantum 
            if (running.quantum_used == RR_TIME) {

                // Log the preemption: RUNNING → READY
                execution_status += print_exec_status(next_time, running.PID, RUNNING, READY);

                running.state = READY; // Move process back to ready queue (time-slice expired)
                running.quantum_used = 0;

                ready_queue.push_back(running);
                sync_queue(job_list, running);

                idle_CPU(running);

                // Re-sort ready queue to maintain priority order
                sort_EP_RR(ready_queue);

                running = ready_queue.front();
                ready_queue.erase(ready_queue.begin());

                running.state = RUNNING;
                running.start_time = next_time;
                running.quantum_used = 0;

                sync_queue(job_list, running);

                // Log the state transition: READY → RUNNING
                execution_status += print_exec_status(next_time, running.PID, READY, RUNNING);

                current_time = next_time;
                continue;
            }

            // Process continues running normally
            sync_queue(job_list, running);
            current_time = next_time;
            continue;
        }

        // If no process is running, simply advance the time (CPU idle cycle)
        current_time++;
    }

    execution_status += print_exec_footer();
    return std::make_tuple(execution_status);
}




int main(int argc, char** argv) {

    //Get the input file from the user
    if(argc != 2) {
        std::cout << "ERROR!\nExpected 1 argument, received " << argc - 1 << std::endl;
        std::cout << "To run the program, do: ./interrutps <your_input_file.txt>" << std::endl;
        return -1;
    }

    //Open the input file
    auto file_name = argv[1];
    std::ifstream input_file;
    input_file.open(file_name);

    //Ensure that the file actually opens
    if (!input_file.is_open()) {
        std::cerr << "Error: Unable to open file: " << file_name << std::endl;
        return -1;
    }

    //Parse the entire input file and populate a vector of PCBs.
    //To do so, the add_process() helper function is used (see include file).
    std::string line;
    std::vector<PCB> list_process;
    while(std::getline(input_file, line)) {
        auto input_tokens = split_delim(line, ", ");
        auto new_process = add_process(input_tokens);
        list_process.push_back(new_process);
    }
    input_file.close();

    //With the list of processes, run the simulation
    auto [exec] = run_simulation(list_process);

    write_output(exec, "execution.txt");

    return 0;
}


