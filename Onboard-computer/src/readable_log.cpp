#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdio.h>

using namespace std;

int main (int argc, char ** argv)
{
    if(argc < 2){
        cout << "no arguments" << endl;
        return -1;
    }
    
    if(argc < 3){
        cout << "not enough arguments" << endl;
        return -1;
    }

    string inputFileName = argv[1];
    string outputDir = argv[2];

    cout << "input file: " << inputFileName << endl << "output dir: " << outputDir << endl;
    
    struct data{
        double time;
        string loglevel;
        string message;
    };

    ifstream inputFile(inputFileName);
    vector<pair<string, vector<data>>> outputData;

    if(inputFile.is_open()){
        double starttime = 0;
        string line;
        string prevNode;

        while(getline(inputFile, line)){
            bool first = true;
            data tmp;
            string substring = line;
            string node;
            u_int8_t state = 0;
            
            // cout << substring << endl << endl;
            
            if(substring.size() > 2){
                if(substring.find("  Node Startup") != string::npos){
                    string tmp = substring.substr(0, substring.find("  Node Startup"));
                    // cout << "string: " << tmp << endl;
                    starttime = stod(tmp);
                    // cout << fixed << "double: " << starttime << endl;
                }
                else{
                    while (substring.find(" ") != string::npos && state < 5){
                        // cout << substring.substr(0, substring.find(" ")) << endl;
                        switch (state)
                        {
                        case 0:{
                            string test = substring.substr(substring.find(" ") + 1);
                            string loglevel = test.substr(0, test.find(" "));
                            if(loglevel == "DEBUG" || loglevel == "INFO" || loglevel == "WARN" || loglevel == "ERROR"){
                                tmp.time = stod(substring.substr(0, substring.find(" "))) - starttime;
                                state = 1;
                            }
                            else{
                                cout << substring << endl;
                                tmp.message = substring;
                                state = 6;
                            }
                            break;
                            }
                        
                        case 1:
                            tmp.loglevel = substring.substr(0, substring.find(" "));
                            state = 2;
                            break;
                        
                        case 2:
                            node = substring.substr(1, substring.find(" ") - 1);
                            state = 3;
                            break;
                        
                        case 3:
                            if(substring.substr(0, substring.find(" ")).find("]") != string::npos){
                                if(!first){
                                    // cout << "yes" << endl;
                                    state = 4;
                                }
                                else{
                                    first = false;
                                }
                            }
                            break;
                        
                        case 4:
                            tmp.message = substring;
                            state = 5;
                            break;
                        
                        default:
                            cout << "I'm not supposed to be here" << endl;
                            break;
                        }


                        substring = substring.substr(substring.find(" ") + 1);
                    }
                    if(state == 4){
                        tmp.message = substring;
                        state = 5;
                    }


                    if(state == 5){
                        bool foundNode = false;
                    
                        for(auto &i: outputData){
                            if(i.first == node){
                                foundNode = true;
                                i.second.push_back(tmp);
                                break;
                            }
                        }
                        if(!foundNode){
                            vector<data> vectortmp(1,tmp);
                            outputData.emplace_back(node, vectortmp);
                        }
                        prevNode = node;
                    }
                    else if(state == 6){
                        for(auto &i: outputData){
                            if(i.first == prevNode){
                                i.second.back().message += " " + tmp.message;
                                break;
                            }
                        }
                    }
                }
            }
            

            // cout << line << endl;
        }
        inputFile.close();
    }
    else{
        cout << "wrong file you idiot" << endl;
    }

    for(auto &i : outputData){
        for(auto j = 0; j < i.first.size() + 4; j ++){
            cout << "#";
        }
        cout << endl << "# " << i.first << " #" << endl;
        for(auto j = 0; j < i.first.size() + 4; j ++){
            cout << "#";
        }
        cout << endl;

        for(auto &j : i.second){
            cout << "time: " << j.time << "\t\tlog level: " << j.loglevel << "\t\tmessage:" << j.message << endl;
        }
        cout << endl;

        string fileName = outputDir + "/" + i.first + ".log";
        ofstream outputFile (fileName);

        if(outputFile.is_open()){
            for(auto &j : i.second){
                outputFile << "time: " << j.time << "\t\tlog level: " << j.loglevel << "\t\tmessage:" << j.message << endl;
            }
            outputFile.close();
        }
        else{
            cout << "yeah i cant open it" << endl;
        }
    }

    return 0;
}