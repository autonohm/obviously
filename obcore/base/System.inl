template <class T>
void System<T>::allocate (unsigned int unRows, unsigned int unCols, T** &array2D)
{
    array2D = new T*[unRows];
    array2D[0] = new T[unRows*unCols];
    for (unsigned int unRow = 1; unRow < unRows; unRow++)
    {
        array2D[unRow] = &array2D[0][unCols*unRow];
    }
}

template <class T>
void System<T>::deallocate (T**& array2D)
{
    delete[] array2D[0];
    delete[] array2D;
    array2D = 0;
}

// SM (9.9.2012): This seems to be buggy, updated (and simplified) version below
/*template <class T>
void System<T>::allocate (unsigned int unRows, unsigned int unCols, unsigned int unSlices, T*** &array3D)
{
    array3D = new T**[unSlices];
    array3D[0] = new T*[unSlices*unCols];
    array3D[0][0] = new T[unSlices*unRows*unCols];
    for (unsigned int unSlice = 0; unSlice < unSlices; unSlice++)
    {
        array3D[unSlice] = &array3D[0][unRows*unSlice];
        for (unsigned int unRow = 0; unRow < unRows; unRow++)
        {
            array3D[unSlice][unRow] = &array3D[0][0][unCols*(unRow+unRows*unSlice)];
        }
    }
}*/

template <class T>
void System<T>::allocate (unsigned int unRows, unsigned int unCols, unsigned int unSlices, T*** &array3D)
{
    array3D = new T**[unRows];
    for (unsigned int unRow = 0; unRow < unRows; unRow++)
    	System<T>::allocate(unCols, unSlices, array3D[unRow]);
}

template <class T>
void System<T>::deallocate (T***& array3D)
{
    delete[] array3D[0][0];
    delete[] array3D[0];
    delete[] array3D;
    array3D = 0;
}
