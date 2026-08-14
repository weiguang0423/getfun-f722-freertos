# S7.2 PC参考链

依赖：Python 3.12、`ai-edge-litert==2.1.6`、`opencv-python-headless==4.13.0.92`。

```powershell
python Linux/RK3568/s7_2/s7_2_pc_reference.py `
  --testset C:\Users\user\Desktop\gesture_testset `
  --detector <hand_detector.tflite> `
  --landmark <hand_landmarks_detector.tflite> `
  --output Linux/RK3568/s7_2/validation/run1
```

模型必须匹配文档36记录的SHA-256。程序使用清单和目录实际媒体文件的并集，避免清单漏项。
重复三次后比较各目录的`results.jsonl`哈希；
`test_s7_2_reference.py`是前后处理最小自检。模型和测试集不复制进飞控仓库。
