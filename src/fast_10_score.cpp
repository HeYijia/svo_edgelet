#include <vector>
#include <climits>
#include <svo/fast.h>

// This is mechanically generated code.
namespace fast
{

static inline bool test_gt_set(int a, int b, int& min_diff)
{
        if(a > b)
        {
                if(a-b < min_diff)
                        min_diff = a-b;

                return 1;
        }
        return 0;
}

inline int fast_corner_score_10(const fast_byte* cache_0, const int offset[], int b)
{
        b++;
        //This function computes the score for a pixel which is known to be
        //a corner at barrier b. So we start looking at b+1 and above to
        //establish where it stops becoming a corner.
        for(;;)
        {
                int cb = *cache_0 + b;
                int c_b= *cache_0 - b;
                int min_diff = INT_MAX;
                if(test_gt_set(*(cache_0 + offset[0]), cb, min_diff))
                 if(test_gt_set(*(cache_0 + offset[8]), cb, min_diff))
                  if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                   if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                          b += min_diff;
                         else
                          if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                           b += min_diff;
                          else
                           break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                         if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                       if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                               if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                b += min_diff;
                               else
                                break;
                              else
                               break;
                            else
                             if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                              if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                              if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                               b += min_diff;
                              else
                               if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                b += min_diff;
                               else
                                break;
                             else
                              if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                               b += min_diff;
                              else
                               break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                             if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                              if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                            else
                             if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                              if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                     else if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                      if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                          else
                           if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                              if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                          else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                           if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                              if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                              if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                     if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                          else
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                         else
                          if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            b += min_diff;
                           else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                          else if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                         else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                          if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                   else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                          else
                           if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                         else
                          if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else
                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                          else
                           if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                         else
                          break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                         if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                  else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                   if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                           b += min_diff;
                          else
                           if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                         else
                          if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else
                    break;
                  else
                   if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                           b += min_diff;
                          else
                           if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                         else
                          if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                      if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                    else
                     break;
                   else
                    break;
                 else if(test_gt_set(c_b, *(cache_0 + offset[8]), min_diff))
                  if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                   if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                           b += min_diff;
                          else
                           if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                         else
                          if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else
                    break;
                  else if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                   if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                     if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                              if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                       if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                              if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                              if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                     else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                      if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                           b += min_diff;
                          else
                           if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                            b += min_diff;
                           else
                            break;
                         else
                          break;
                        else
                         if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                     else
                      if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                    else
                     if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                   else
                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                  else
                   if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else
                    break;
                 else
                  if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                   if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                           b += min_diff;
                          else
                           if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                         else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                          if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        break;
                      else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                       if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                     else
                      break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                     if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                         if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                   else
                    break;
                  else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                   if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else
                    break;
                  else
                   if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else
                    break;
                else if(test_gt_set(c_b, *(cache_0 + offset[0]), min_diff))
                 if(test_gt_set(*(cache_0 + offset[8]), cb, min_diff))
                  if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                   if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                           b += min_diff;
                          else
                           if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                         else
                          if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else
                    break;
                  else if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                   if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                          else
                           break;
                         else
                          if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                       if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                           b += min_diff;
                          else
                           break;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                        else
                         if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                         if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                      else
                       break;
                     else
                      break;
                    else
                     if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                   else
                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                  else
                   if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                          else
                           if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                         else
                          break;
                        else
                         if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else
                    break;
                 else if(test_gt_set(c_b, *(cache_0 + offset[8]), min_diff))
                  if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                   if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                          if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                           b += min_diff;
                          else
                           if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                         else
                          break;
                        else
                         if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                     else
                      break;
                    else
                     break;
                   else
                    break;
                  else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                   if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                           if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                          else
                           if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                             b += min_diff;
                            else
                             break;
                          else
                           if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                     if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                             b += min_diff;
                            else
                             if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                              if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                           else
                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                              if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                            else
                             break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                      if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                             b += min_diff;
                            else
                             if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                              if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                               if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                b += min_diff;
                               else
                                break;
                              else
                               break;
                             else
                              break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                       if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                          b += min_diff;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                           b += min_diff;
                          else
                           break;
                        else
                         if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                              if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                               if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                b += min_diff;
                               else
                                break;
                              else
                               break;
                             else
                              break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                             b += min_diff;
                            else
                             if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                              if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                               if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                b += min_diff;
                               else
                                break;
                              else
                               break;
                             else
                              break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                     else
                      if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                           if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                              if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                            else
                             break;
                           else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                             b += min_diff;
                            else
                             if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                              if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                           else
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                              if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                               b += min_diff;
                              else
                               break;
                             else
                              break;
                            else
                             break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                    else
                     if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                         if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                          if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                           if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                          else
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                   else
                    if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                             if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                          else
                           if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                  else
                   if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                         if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                           b += min_diff;
                          else
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            b += min_diff;
                           else
                            break;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else
                    break;
                 else
                  if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                   if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                      if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                     else
                      break;
                    else
                     break;
                   else if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                      if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                         if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                           b += min_diff;
                          else
                           if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                            b += min_diff;
                           else
                            break;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        break;
                      else
                       if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                     else
                      break;
                    else
                     if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                   else
                    if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                       if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                  else
                   break;
                else
                 if(test_gt_set(*(cache_0 + offset[8]), cb, min_diff))
                  if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                   if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                           b += min_diff;
                          else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                           if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                         else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                          if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       break;
                     else
                      break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                     if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                            b += min_diff;
                           else
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                          else
                           if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                         else
                          break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                            b += min_diff;
                           else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                           else
                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                             if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                              b += min_diff;
                             else
                              break;
                            else
                             break;
                          else
                           break;
                         else
                          break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                         if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                   else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else
                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                     if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                      if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                       if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                         if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                          if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                           if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                  else
                   break;
                 else if(test_gt_set(c_b, *(cache_0 + offset[8]), min_diff))
                  if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                   if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                   else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                     if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                       if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       break;
                     else if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                       if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                         if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                           b += min_diff;
                          else
                           if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                            b += min_diff;
                           else
                            break;
                         else
                          if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                        else
                         break;
                       else
                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       break;
                     else
                      if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                       if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       break;
                    else
                     break;
                   else
                    if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                     if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                      if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                       if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                            b += min_diff;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                       else
                        if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                         if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                          if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                           if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                             b += min_diff;
                            else
                             break;
                           else
                            break;
                          else
                           break;
                         else
                          break;
                        else
                         break;
                      else
                       break;
                     else
                      break;
                    else
                     break;
                  else
                   break;
                 else
                  break;

        }

        return b-1;
}

void fast_corner_score_10(
  const fast_byte* img,
  const int img_stride,
  const std::vector<fast_xy>& corners,
  const int threshold,
  std::vector<int>& scores)
{
  scores.resize(corners.size());
  int pixel[16] = {
    0 + img_stride * 3,
    1 + img_stride * 3,
    2 + img_stride * 2,
    3 + img_stride * 1,
    3 + img_stride * 0,
    3 + img_stride * -1,
    2 + img_stride * -2,
    1 + img_stride * -3,
    0 + img_stride * -3,
    -1 + img_stride * -3,
    -2 + img_stride * -2,
    -3 + img_stride * -1,
    -3 + img_stride * 0,
    -3 + img_stride * 1,
    -2 + img_stride * 2,
    -1 + img_stride * 3,
  };
  for(unsigned int n=0; n < corners.size(); n++)
    scores[n] = fast_corner_score_10(img + corners[n].y*img_stride + corners[n].x, pixel, threshold);
}

float shiTomasiScore(const cv::Mat& img, int u, int v)
{
  assert(img.type() == CV_8UC1);

  float dXX = 0.0;
  float dYY = 0.0;
  float dXY = 0.0;
  const int halfbox_size = 4;
  const int box_size = 2*halfbox_size;
  const int box_area = box_size*box_size;
  const int x_min = u-halfbox_size;
  const int x_max = u+halfbox_size;
  const int y_min = v-halfbox_size;
  const int y_max = v+halfbox_size;

  if(x_min < 1 || x_max >= img.cols-1 || y_min < 1 || y_max >= img.rows-1)
    return 0.0; // patch is too close to the boundary

  const int stride = img.step.p[0];
  for( int y=y_min; y<y_max; ++y )
  {
    const uint8_t* ptr_left   = img.data + stride*y + x_min - 1;
    const uint8_t* ptr_right  = img.data + stride*y + x_min + 1;
    const uint8_t* ptr_top    = img.data + stride*(y-1) + x_min;
    const uint8_t* ptr_bottom = img.data + stride*(y+1) + x_min;
    for(int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
    {
      float dx = *ptr_right - *ptr_left;
      float dy = *ptr_bottom - *ptr_top;
      dXX += dx*dx;
      dYY += dy*dy;
      dXY += dx*dy;
    }
  }

  // Find and return smaller eigenvalue:
  dXX = dXX / (2.0 * box_area);
  dYY = dYY / (2.0 * box_area);
  dXY = dXY / (2.0 * box_area);
  return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
}
} // namespace Fast
