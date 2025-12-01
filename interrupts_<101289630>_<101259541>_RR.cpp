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

std::tuple<std::string /* add std::string for bonus mark */ > run_simulation(std::vector<PCB> list_processes) {

    std::vector<PCB> ready_queue;   //The ready queue of processes
    std::vector<PCB> wait_queue;    //The wait queue of processes
    std::vector<PCB> job_list;      //A list to keep track of all the processes. This is similar
                                    //to the "Process, Arrival time, Burst time" table that you
                                    //see in questions. You don't need to use it, I put it here
                                    //to make the code easier :).

    unsigned int current_time = 0;
    PCB running;
    unsigned int RR_TIME = 100; // Declaring Round Robin with a 100 ms timeout

    //Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;

    //make the output table (the header row)
    execution_status = print_exec_header();

    //Loop while till there are no ready or waiting processes.
    //This is the main reason I have job_list, you don't have to use it.
    while(!all_process_terminated(job_list) || job_list.empty()) {

        //Inside this loop, there are three things you must do:
        // 1) Populate the ready queue with processes as they arrive
        //Go through the list of proceeses
        for (auto &process : list_processes) {
            if (process.arrival_time == current_time) { // check if the AT = current time

                PCB p = process;
                assign_memory(p); // if so, assign memory and put the process into the ready queue

                p.state = READY;  // Set the process state to READY
                p.cpu_executed = 0;
                p.quantum_used = 0;
                p.wait_until = 0;

                ready_queue.push_back(p); // Add the process to the ready queue
                job_list.push_back(p); // Add it to the list of processes

                execution_status += print_exec_status(current_time, p.PID, NEW, READY);
            }
        }

        ///////////////////////MANAGE WAIT QUEUE/////////////////////////
        //This mainly involves keeping track of how long a process must remain in the ready queue
        // 2) Manage the wait queue
        for (auto it = wait_queue.begin(); it != wait_queue.end();) {
            if (it->wait_until == current_time) {

                PCB p = *it;
                p.state = READY;

                ready_queue.push_back(p);
                sync_queue(job_list, p);

                execution_status += print_exec_status(current_time, p.PID, WAITING, READY);

                it = wait_queue.erase(it);

            } else {
                ++it;
            }
        }

        //////////////////////////SCHEDULER//////////////////////////////
        // 3) Schedule processes from the ready queue
        if (running.PID == -1 && !ready_queue.empty()) {

            running = ready_queue.front();
            ready_queue.erase(ready_queue.begin());

            running.start_time = current_time;
            running.state = RUNNING;

            sync_queue(job_list, running);

            execution_status += print_exec_status(current_time, running.PID, READY, RUNNING);
        }


        // Execute the currently running process 
        if (running.PID != -1 && running.state == RUNNING) {

            running.remaining_time--;
            running.cpu_executed++;
            running.quantum_used++;

            unsigned int new_time = current_time + 1;

            // Check if process has completed all of its execution
            if (running.remaining_time == 0) {
                // Log the state transition from RUNNING to TERMINATED
                execution_status += print_exec_status(new_time, running.PID, RUNNING, TERMINATED);

                terminate_process(running, job_list);
                idle_CPU(running);

                // Try immediate dispatch, meaning if ready queue has processes, schedule the next one
                if (!ready_queue.empty()) {
                    // Take the first process from ready queue
                    running = ready_queue.front();
                    ready_queue.erase(ready_queue.begin());
                    
                    // Initialize the newly dispatched process
                    running.start_time = new_time;
                    running.state = RUNNING;

                    sync_queue(job_list, running);
                    // Log the state transition from READY to RUNNING
                    execution_status += print_exec_status(new_time, running.PID, READY, RUNNING);
                }

                current_time = new_time;
                continue; // Skip to next iteration
            }

            // Check if process needs to perform I/O operation
            if (running.io_freq > 0 && running.cpu_executed == running.io_freq) {
                // Log the state transition from RUNNING to WAITING
                execution_status += print_exec_status(new_time, running.PID, RUNNING, WAITING);

                PCB w = running;
                w.state = WAITING;
                w.cpu_executed = 0; // Reset CPU execution counter for next CPU burst
                w.quantum_used = 0; // Reset quantum counter since it's leaving CPU
                w.wait_until = new_time + w.io_duration; // Calculate when I/O will complete

                // Move process to wait queue and update job_list
                wait_queue.push_back(w);
                sync_queue(job_list, w);

                idle_CPU(running);

                // Dispatch next process if ready queue is not empty
                if (!ready_queue.empty()) {

                    // Take the first process from ready queue
                    running = ready_queue.front();
                    ready_queue.erase(ready_queue.begin());

                    // Initialize the newly dispatched process
                    running.start_time = new_time;
                    running.state = RUNNING;

                    sync_queue(job_list, running);

                    // Log the state transition from READY to RUNNING
                    execution_status += print_exec_status(new_time, running.PID, READY, RUNNING);
                }

                current_time = new_time;
                continue; // skip to next iteration
            }

            // Check if the process has used up its time quantum
            if (running.quantum_used == RR_TIME) {

                // Preempt the running process: log RUNNING → READY transition
                execution_status += print_exec_status(new_time, running.PID, RUNNING, READY);

                // Move the preempted process back to ready queue
                running.state = READY;
                running.quantum_used = 0; 
                ready_queue.push_back(running); // Add to end of ready queue
                sync_queue(job_list, running);

                idle_CPU(running);

                // Dispatch new process at same timestamp 
                if (!ready_queue.empty()) {
                    
                    // Take the first process from ready queue
                    running = ready_queue.front();
                    ready_queue.erase(ready_queue.begin());

                    // Initialize the newly dispatched process
                    running.start_time = new_time;
                    running.state = RUNNING;
                    running.quantum_used = 0;

                    sync_queue(job_list, running);

                    execution_status += print_exec_status(new_time, running.PID, READY, RUNNING);
                }

                current_time = new_time;
                continue;
            }

            // Process continues running
            // Update job_list with current running process state and advance time
            sync_queue(job_list, running);
            current_time = new_time;
            continue;
        }

        // If no process is running, advance the time
        current_time++;
    }
    
    //Close the output table
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