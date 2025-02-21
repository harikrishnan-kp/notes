
# memory management in C
compiler convert c source into obj. it seperate code into sections .text .bss etc(depends on file format)

- Static memory allocation
- Automatic memory allocation
- Dynamic memory allocation
## Static memory allocation
the process of memory allocation for a program during the build process is called as static memory allocation.
- we have to declare the required size of memory for each thing we are using,
- once declared we can`t change it during execution time.
### disadvantages
- consider a situation in which we need to store some random no.of elements in a array during run time. we have no idea about the number of elements and the size of array required to store the elements.but we have to declare the array.
- if we declare array with size less than requirement stack over flow occurs, if the size of array is larger than requirments we are wasting memory 
## dynamic memory alloacation 
The process of allocating memory during the execution time is called as dynamic memory allocation
- this will overcome disadvantages of static memory allocation
- this method use heap
- there are some built in funtions for this operation
    - malloc()
    - calloc()
    - realloc()
    - free()
- we need pointers to access this dynamicallly allocated memory