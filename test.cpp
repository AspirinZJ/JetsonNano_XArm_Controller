#include <bits/stdc++.h>

// Fucntion to convert a string to integer array
void convertStrtoArr(std::string str, uint16_t *arr)
{
    int j = 0, i, sum = 0;

    for (i = 0; str[i] != '\0'; i++)
    {
        if (str[i] == ' ')
        {
            j++;
        }
        else
        {
            arr[j] = arr[j] * 10 + (str[i] - 48);
        }
    }
}

// Driver code
int main()
{
    std::string str = "2 6 3 14";
    uint16_t array[4] = {0};
    convertStrtoArr(str, array);
    for (size_t i = 0; i < 4; i++)
    {
        std::cout << array[i] << ' ';
        std::cout << std::endl;
    }
    return 0;
}