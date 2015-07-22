//vector.h file 
#ifndef VECTOR_H
#define VECTOR_H

template <class T> //using templates, which they can substitute for int, double, etc. depending on their function.
class vector       //creating a vector class
{
public:
typedef T * iterator;
    
	// constructors
vector() {buffer = 0; reserve(0);}			
vector(unsigned int size) {buffer = 0; resize(size);}
vector(unsigned int size, T initial)
    {buffer = 0; resize(size); fill(begin(), end(), initial);}
vector(vector & v)
    {buffer = 0; resize(v.size()); copy(v.begin(), v.end(), begin());}
~vector() {delete buffer;}
    
	// member functions
T	back()      {return buffer[mySize-1];}
iterator      begin()     {return buffer;}
int	capacity()  {return myCapacity;}
iterator	end()       {return buffer + mySize;}
bool	empty()     {return mySize == 0;}
T	front()     {return buffer[0];}
void	pop_back()  {mySize--;}
void	push_back(T value);
void	reserve(unsigned int newSize);
void	resize(unsigned int newSize)	{reserve(newSize); mySize = newSize;}
int	size()      {return mySize;}

    // operators
T &	operator[ ](unsigned int index) {return buffer[index];}
private:
unsigned int mySize;
unsigned int myCapacity;
T * buffer;
};



template <class T> void vector<T>::reserve(unsigned int newCapacity)  //reserve capacity at leaset as large as argument value
{
if (buffer == 0)			    
  {					    //*No buffer, zero size
    mySize = 0;				    
    myCapacity = 0;                         
  }
					   //don't do anything if already large enough
if (newCapacity <= myCapacity)             //compares newCapacity to myCapacity if it does not meet the
return;                                    //argument then it will continue
					   //*Allocate new buffer, make sure successful
T * newBuffer = new T [newCapacity];
copy(buffer, buffer + mySize, newBuffer);  //copy this sequence (start at buffer, stop at buffer + mySize, 
                                           //destination is newBuffer) into newBuffer (resets data field)
myCapacity = newCapacity;
delete buffer;				   //deletes the buffer 
buffer = newBuffer;			   //assigns buffer to newBuffer 
}



template <class T> void vector<T>::push_back (T value)  //*Push value on to end of vector
{					  //*Grow buffer if necessary
if (mySize >= myCapacity)		  //if myCapacity is greater than or equal to mySize
reserve(myCapacity + 5);		  //set reserve to myCapacity + 5
buffer[mySize++] = value;
}

# endif

