/**
 * @file differ_amp.c
 * @brief 基于差分法的三角波幅度测量实现。
 */

#include "differ_amp.h"

#include <stdint.h>
#define C (3.3/4096)

float32_t median_differ = 0, average = 0;
float differ_data[1023];
float abs_differ_data[1023];
float AD_value1[1024];
uint16_t cnt_up = 0, cnt_down = 0;
uint16_t idx_up[1023], idx_down[1023];




// 使用非递归方式将差分幅值按升序排列。
void quick_sort_large(float32_t* arr, uint32_t length) {

    typedef struct {
        uint32_t low;
        uint32_t high;
    } StackItem;
    StackItem stack[64];
    int top = -1;

    stack[++top] = (StackItem){0, length - 1};

    while (top >= 0) {
        StackItem current = stack[top--];
        uint32_t low = current.low, high = current.high;

        if (high - low < 32) {
            for (uint32_t i = low + 1; i <= high; i++) {
                float32_t key = arr[i];
                int32_t j = i - 1;
                while (j >= (int32_t)low && arr[j] > key) {
                    arr[j + 1] = arr[j];
                    j--;
                }
                arr[j + 1] = key;
            }
            continue;
        }

        uint32_t mid = low + (high - low) / 2;
        float32_t pivot = (arr[low] + arr[mid] + arr[high]) / 3.0f;

        uint32_t i = low, j = high;
        while (i <= j) {
            while (arr[i] < pivot) i++;
            while (arr[j] > pivot) j--;
            if (i <= j) {
                float32_t temp = arr[i];
                arr[i++] = arr[j];
                arr[j--] = temp;
            }
        }

        if (j - low < high - i) {
            if (low < j) stack[++top] = (StackItem){low, j};
            if (i < high) stack[++top] = (StackItem){i, high};
        } else {
            if (i < high) stack[++top] = (StackItem){i, high};
            if (low < j) stack[++top] = (StackItem){low, j};
        }
    }
}


float Differ_Tri_Amp(uint16_t Length, uint16_t *AD_value)
{
	uint16_t i,j,k,t;
//	uint16_t cnt_up = 0, cnt_down = 0;
//	uint16_t idx_up[Length-1], idx_down[Length-1];
	float32_t Amp = 0;

//
//	for(i=0;i<Length-1;i++)
//	{
//		idx_up[i]=0;
//		idx_down[i]=0;
//
//	}
	// 计算差分前先去除输入信号的直流偏置。
	for(i=0;i<Length;i++)
	{
		average+=AD_value[i];
	}
	average=average/Length;

	for(i=0;i<Length;i++)
	{
		AD_value1[i] = ((float)AD_value[i]-average)*C;
	}

		median_differ = 0;

	// 计算相邻样本之差及其绝对值。
	for(i=0;i<Length-1;i++)
	{
		differ_data[i]=AD_value1[i+1]-AD_value1[i];


		if(differ_data[i]<0)
		{
			abs_differ_data[i]=-differ_data[i];
		}
		else
		{
			abs_differ_data[i]=differ_data[i];
		}
	}

	// 对差分幅值排序，用于得到较稳健的差分阈值。
	quick_sort_large(abs_differ_data,Length-1);


	// 对排序后较大的一半差分幅值取平均。
	for(t=(Length-1)/2;t<Length-1;t++)
	{
		median_differ = median_differ + abs_differ_data[t];
	}
	median_differ = median_differ/(Length/2);


	// 在差分波形中定位候选极值点。
	for ( j = 1; j < Length-2 ; j++)
	{
        if ((differ_data[j] < median_differ/2 && differ_data[j]>0 )||(differ_data[j]>-median_differ/2 && differ_data[j]<0))
				{
						if (differ_data[j] > differ_data[j-1] && differ_data[j+1] > differ_data[j])
							{

                idx_up[cnt_up] = j;
								cnt_up++;

            }
						else if (differ_data[j] < differ_data[j-1] && differ_data[j+1] < differ_data[j])

						{
              idx_down[cnt_down] = j;
							cnt_down++;
            }
        }
    }

		// 累加已检测极值点对应的幅度贡献。
	for(k=0;k<cnt_up;k++)
	{
		Amp = Amp + (median_differ- AD_value1[idx_up[k]] - AD_value1[idx_up[k]+1])/2;
		idx_up[k]=0;
	}
	for(k=0;k<cnt_down;k++)
	{
		Amp = Amp + (median_differ +AD_value1[idx_down[k]]+ AD_value1[idx_down[k]+1])/2;
		idx_down[k]=0;
	}

	if((cnt_down + cnt_up) > 0)
	{
    Amp = 2*Amp/(cnt_down + cnt_up);
	} else
	{
    Amp = 0;
	}
	cnt_down=0;
	cnt_up=0;
	return  Amp ;

}
