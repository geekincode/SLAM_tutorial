#include <iostream>
#include <string>
#include <vector>
using namespace std;


template <class T>
class A
{
public:
    A(T t = 0){
        this->t = t;
    }

    T& get(){
        return t;
    }

private:
    T t;
};

void printA(A<int>& a){
    cout << a.get() << endl;
}

int main(int argc, char **argv)
{

    cout << argc << endl;
    for (int i = 0; i < argc; i++)
    {
        cout << argv[i] << endl;
    }

    if (argc > 1) {
        A<int> a(std::stoi(argv[1]));
        cout << a.get() << endl; 
        printA(a);
    }
    return 0; 
}