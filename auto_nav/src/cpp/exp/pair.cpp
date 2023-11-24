#include<bits/stdc++.h> 
using namespace std; 

// // A Method that returns multiple values using 
// // tuple in C++. 
// tuple<int, int, char> foo(int n1, int n2) 
// { 
// 	// Packing values to return a tuple 
// 	return make_tuple(n2, n1, 'a');			 
// } 

// // A Method returns a pair of values using pair 
// std::pair<int, int> foo1(int num1, int num2) 
// { 
// 	// Packing two values to return a pair 
// 	return std::make_pair(num2, num1);			 
// } 

// std::pair<double, double> foo2()
// {	double d = 10.0;
// 	return std::make_pair(d,d);
// }

// int main() 
// { 
// 	int a,b; 
// 	char cc; 
	
// 	// Unpack the elements returned by foo 
// 	tie(a, b, cc) = foo(5, 10);	 
	
// 	// Storing returned values in a pair 
// 	pair<int, int> p = foo1(5,2); 
	
// 	cout << "Values returned by tuple: "; 
// 	cout << a << " " << b << " " << cc << endl; 
	
// 	cout << "Values returned by Pair: "; 
// 	cout << p.first << " " << p.second; 
// 	pair<double, double> d;
// 	d= foo2();
// 	cout<<d.first<<d.second;



// 	return 0; 
// } 


int main()
{
	vector<int> v;
	for(int i = 0; i<10;i++)
		v.push_back(i);
	v.erase(v.begin());
	v.erase(v.begin());
	for(int i = 0; i < v.size(); i++)
	{
		cout<<"  "<< v.at(i);
	}
}