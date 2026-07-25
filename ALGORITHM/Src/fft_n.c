/**
 * @file fft_n.c
 * @brief 原位基 2 复数 FFT 实现。
 */

#include "fft_n.h"
/* 对应当前 FFT 长度的旋转因子查找表。 */
float32_t   costab[MAX_FFT_N/2];
float32_t   sintab[MAX_FFT_N/2];
void InitTableFFT(uint32_t n)
{
	uint32_t i;

	/* 预先计算旋转因子的正弦和余弦值。 */

	for (i = 0; i < n/2; i ++ )
	{
		sintab[ i ]=  sin( 2 * PI * i / MAX_FFT_N );
		costab[ i ]=  cos( 2 * PI * i / MAX_FFT_N );
	}

}
/* 返回 floor(log2(m))；输入为 0 时不存在有效指数。 */
int find_exponent(unsigned int m) {

	  int exponent = 0;
	if (m == 0) {
        return -1;
    }


    while (m> 1) {
        m >>= 1;
        exponent++;
    }
    return exponent;
}


/* 使用位倒序输入的迭代基 2 复数 FFT。 */
int index1;
void cfft(struct compx *_ptr, uint32_t FFT_N )
{
	float32_t TempReal1, TempImag1, TempReal2, TempImag2;
	uint32_t k,i,j,z;
	uint32_t Butterfly_NoPerColumn;				    /* 本级的蝶形分组数量。 */
	uint32_t Butterfly_NoOfGroup;					/* 当前蝶形分组的序号。 */
	uint32_t Butterfly_NoPerGroup;					/* 每组内配对样本的偏移量。 */
	uint32_t ButterflyIndex1,ButterflyIndex2,P,J;
	uint32_t L;
	uint32_t M;
  index1 = find_exponent(FFT_N);
	z=FFT_N/2;                  					/* 位倒序寻址的初始掩码。 */
	for(i=0,j=0;i<FFT_N-1;i++)
	{
		/* 进入各级 FFT 运算前，先将输入样本按位倒序排列。 */
		if(i<j)
		{
			TempReal1  = _ptr[j].real;
			_ptr[j].real= _ptr[i].real;
			_ptr[i].real= TempReal1;
		}

		k=z;                    				  /* 开始位倒序进位传播。 */

		while(k<=j)               				  /* 清除倒序索引中末尾连续的置位比特。 */
		{
			j=j-k;                 				  /* 清除当前位置的索引位。 */
			k=k/2;                 				  /* 进位移动到下一位。 */
		}

		j=j+k;                   				  /* 将第一个被清除的倒序位设为1。 */
	}

	/* 每一级基 2 运算都将分组数减半，并将组宽加倍。 */
	/* Butterfly_NoPerColumn 表示当前级的分组数量。 */
	/* Butterfly_NoPerGroup 表示一对蝶形样本之间的间隔。 */
	Butterfly_NoPerColumn = FFT_N;
	Butterfly_NoPerGroup = 1;
	M =index1;
	for ( L = 0;L < M; L++ )
	{
		Butterfly_NoPerColumn >>= 1;		/* N=8 时，分组数依次为 4, 2, 1。 */

		/* 处理当前级中的每个蝶形分组。 */
		for ( Butterfly_NoOfGroup = 0;Butterfly_NoOfGroup < Butterfly_NoPerColumn;Butterfly_NoOfGroup++ )
		{
			for ( J = 0;J < Butterfly_NoPerGroup;J ++ )	    /* 组内配对样本的偏移。 */
			{					   						    /* 执行一次基 2 蝶形运算。 */
				ButterflyIndex1 = ( ( Butterfly_NoOfGroup * Butterfly_NoPerGroup ) << 1 ) + J;/* (0,2,4,6)(0,1,4,5)(0,1,2,3) */
				ButterflyIndex2 = ButterflyIndex1 + Butterfly_NoPerGroup;/* 蝶形的第二个输入索引。 */
				P = J * Butterfly_NoPerColumn;				/* 本级旋转因子的查表索引。 */

				/* 用旋转因子对第二个样本进行复数旋转。 */
				TempReal2 = _ptr[ButterflyIndex2].real * costab[ P ] +  _ptr[ButterflyIndex2].imag * sintab[ P ];
				TempImag2 = _ptr[ButterflyIndex2].imag * costab[ P ] -  _ptr[ButterflyIndex2].real * sintab[ P ] ;
				TempReal1 = _ptr[ButterflyIndex1].real;
				TempImag1 = _ptr[ButterflyIndex1].imag;

				/* 保存蝶形运算的和、差输出。 */
				_ptr[ButterflyIndex1].real = TempReal1 + TempReal2;
				_ptr[ButterflyIndex1].imag = TempImag1 + TempImag2;
				_ptr[ButterflyIndex2].real = TempReal1 - TempReal2;
				_ptr[ButterflyIndex2].imag = TempImag1 - TempImag2;
			}
		}

		Butterfly_NoPerGroup<<=1;							/* 下级的配对间距翻倍。 */
	}
}
