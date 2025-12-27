## 简介
智能车5g组树莓派里的代码，主要包括图像处理的控制系统

## 任务
1. 移植 linux 端的图像处理的循迹代码到trace.cpp里面，并加入调试功能来查看图像。 注意你只需要关注循迹部分的处理，元素处理请忽略！
2. 重点关注image_Q里面的 Earge_Search_Mid 函数，这个是处理边界的关键函数。
3. 这个工程有相当多的不友好的命名，包括拼音和英文混合，你可以建立一份映射表来更好的思考。
4. 目前的问题是image_Q里面的循迹函数无法处理杂线，导致小车在赛道上遇到直跑到与弯跑道相交的地方会跑偏。下面这份代码可能会有所改善。
5. 你在移植到trace.cpp的时候需要把下面的代码也移植进去并设置一个模式来切换，方便我们调试。
6. 注意，Earge_Search_Mid 在win的编译器可以正常编译，但是在运行的时候会报错，这一点需要你修复。

for (j = Mid; j >= 10; j-=4)                         //以前一行中点为起点向左查找边界
   {
   if (( data.at <uchar>(i, j) < 100)&&( data.at <uchar>(i, j-4) < 100)&&( data.at <uchar>(i, j-8) > 100))                     /*黑白 ?*//*为啥不用全用阈值写if(data[i][j] < BlackThres  && data[i][j-1] < BlackThres) ?*/
   {/*上面的BlackThres后面的数字可以根据需要调一 ?*/
   if(j>= 320/2 + 50)
   {
   j = j-30;
   continue;
   //继续 ?
   }
   else
   {
   left_candidates.push_back(j);
   // 继续搜索更左边的边界
   continue;
   /*Left_Add_Flag[i] = 0;           //左边界不需要补线，清除标志 ?
   Left_Line[i] = j;       //记录当前j值为本行实际左边 ?
   Left_Add_Line[i] = j;           //记录实际左边界为补线左边 ?
   break;  */
   }
   //找到退 ?
   }
   }
   /*右边线查 ?*/
   for (j = Mid; j <= COL - 10; j+=4)    // 以前一行中点为起点向右查找右边 ?
   {
   if (( data.at <uchar>(i, j) < 100)&&( data.at <uchar>(i, j+4) < 100)&&( data.at <uchar>(i, j+8) > 100))
   {/*上面的BlackThres后面的数字可以根据需要调一 ?*/
   if(j<= 320/2 - 50)
   {
   j = j+30;
   continue;
   //继续 ?
   }
   else
   {
   right_candidates.push_back(j);
   // 继续搜索更右边的边界
   continue;
   /*Right_Add_Flag[i] = 0;      //右边界不需要补线，清除标志 ?
   Right_Line[i] = j;      //记录当前j值为本行右边 ?
   Right_Add_Line[i] = j;      //记录实际右边界为补线左边 ?
   break;    */
   }                                 //找到退 ?
   }
   }
   if (!left_candidates.empty())
   {
   // 找到最左边的边界（数值最小的）
   int16 best_left = *min_element(left_candidates.begin(), left_candidates.end());
   Left_Add_Flag[i] = 0;           // 找到真实边界
   Left_Line[i] = best_left;
   Left_Add_Line[i] = best_left;

        // 可选：打印调试信息
        // if(left_candidates.size() > 1) {
        //     printQ("多左边界:", left_candidates.size());
        // }
   }

   if (!right_candidates.empty())
   {
   // 找到最右边的边界（数值最大的）
   int16 best_right = *max_element(right_candidates.begin(), right_candidates.end());
   Right_Add_Flag[i] = 0;          // 找到真实边界
   Right_Line[i] = best_right;
   Right_Add_Line[i] = best_right;

        // 可选：打印调试信息
        // if(right_candidates.size() > 1) {
        //     printQ("多右边界:", right_candidates.size());
        // }
   }

   }