#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"

int main(void)
{
    FILE* file = fopen("data.json", "r");
    if (file == NULL)
    {
        printf("Failed to open data.json\n");
        return -1;
    }

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    char* jsondata = (char*)malloc(file_size + 1);
    fread(jsondata, sizeof(char), file_size, file);
    jsondata[file_size] = '\0';
    fclose(file);

    cJSON* root = cJSON_Parse(jsondata);
    if (root == NULL)
    {
        printf("Failed to parse the JSON data.\n");
        free(jsondata);
        return -1;
    }

    cJSON* name = cJSON_GetObjectItemCaseSensitive(root, "name");
    if (cJSON_IsString(name) && (name->valuestring != NULL))
    {
        printf("Name = %s\n", name->valuestring);
    }

    cJSON* age = cJSON_GetObjectItemCaseSensitive(root, "age");
    if (cJSON_IsNumber(age))
    {
        printf("Age : %d\n", age->valueint);
    }

    cJSON* hobbies = cJSON_GetObjectItemCaseSensitive(root, "hobbies");
    if (cJSON_IsArray(hobbies))
    {
        printf("Hobbies:\n");
        cJSON* hobby;
        cJSON_ArrayForEach(hobby, hobbies)
        {
            if (cJSON_IsString(hobby) && (hobby->valuestring != NULL))
            {
                printf("- %s\n", hobby->valuestring);
            }
        }
    }

    cJSON_Delete(root);
    free(jsondata);

    return 0;
}