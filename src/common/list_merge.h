#include <list>

template <class T> 
void merge_list(std::list<T> &a, std::list<T> &b){
    auto a_it = a.begin();
    auto b_it = b.begin();
    while(a_it != a.end() && b_it != b.end()){
        if(*a_it >= *b_it){
            a.insert(a_it, *b_it);
            b_it++;
        }
        else{
            a_it++;
        }
    }
    if(a_it == a.end()){
        a.insert(a_it, b_it, b.end());
    }
}
