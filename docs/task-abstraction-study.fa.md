# بررسی انتزاع تسک‌های Nav2، DAG و پردازنده‌های مدل‌شده

تاریخ بررسی: ۲۰۲۶-۱۰-۰۹. مبنا: همین پیست و دور موفقِ commit `3f11a7e`، ROS 2 Jazzy، Nav2 1.3.13 و rclcpp 28.1.22. بررسی تنظیمات، سورس رسمیِ tag همان نسخه، headerهای نصب‌شده و گراف زنده انجام شد. هدف جدیدی برای حرکت ارسال نشد، پکیجی نصب نشد و الگوریتم، ساعت، منابع و پارامترهای اجرای ربات تغییر نکردند. موارد پیشنهادی زیر هنوز پیاده‌سازی نشده‌اند.

## نتیجهٔ فنی

ایدهٔ اجرای واقعیِ الگوریتم‌های موجود روی لپ‌تاپ، همراه با زمان‌بندی و تحویل نتایج در زمانِ پردازندهٔ مدل‌شده، قابل پیاده‌سازی است. این کار یک هم‌شبیه‌سازیِ کنترل‌شدهٔ نرم‌افزار و محیط فیزیکی است؛ Gazebo به‌تنهایی چنین پردازنده‌ای فراهم نمی‌کند. انتخاب مرز job و معنای وابستگی/صف لازم است، ولی نیازی به ساختن الگوریتم ناوبری یا workload ساختگی نداریم.

از نصب فعلی می‌توان نوع اجزا، نرخ‌های اسمی، منابع ورودی، برخی شرط‌های فعال شدن، ساختار الگوریتم و محدودیت‌های زمان‌دار را خواند. هنوز CPU demand، WCET، release jitter، blocking، DAG علّیِ jobها یا زمانِ اجرای همین کارها روی Cortex-A15/A7 را نداریم. این‌ها از مدت فیلم یا مدت پیمایش قابل استنتاج نیستند.

## ۱. آنچه از قبل داریم

| واحد واقعیِ کار | نرخ/تناوب اسمیِ موجود | نوع آزادسازی و ساعتِ کد فعلی | نکتهٔ مدل‌سازی |
|---|---|---|---|
| تولید LiDAR در Gazebo | ۵ Hz؛ ۲۰۰ ms | ساعت شبیه‌سازی | منبع ورودی خارجی برای مدل CPU؛ الگوریتمِ AMCL نیست |
| تولید odometry/TF حرکت | ۳۰ Hz؛ حدود ۳۳٫۳ ms | ساعت شبیه‌سازی | منبع دادهٔ وضعیت و سرعت |
| دریافت scan در AMCL | با رسیدن scan و مهیا شدن TF | پیام‌محور | محاسبهٔ سنگین فیلتر در هر scan اجباری نیست |
| یک دور به‌روزرسانی local costmap | ۵ Hz؛ ۲۰۰ ms | thread مستقل و `WallRate` | دادهٔ scan/TF ذخیره‌شده را مصرف می‌کند؛ publication جداگانه ۲ Hz است |
| یک دور به‌روزرسانی global costmap | ۱ Hz؛ ۱۰۰۰ ms | thread مستقل و `WallRate` | مسیر را خودبه‌خود هر بار تولید نمی‌کند |
| درخواست ComputePathThroughPoses | محدودکنندهٔ نرخ ۱ Hz در BT سناریو | event/action؛ زمان host در RateController | دورهٔ سخت ۱۰۰۰ ms تضمین نمی‌شود؛ نتیجه، شکست و recovery در آزادسازی اثر دارند |
| یک iteration کنترل MPPI | ۲۰ Hz؛ ۵۰ ms اسمی | حلقهٔ `WallRate` داخل workerِ FollowPath، فقط هنگام مأموریت | FollowPath کامل یک job پنج‌دقیقه‌ای نیست؛ iteration واحد مناسب‌تری است |
| تولید نویز MPPI | تحریک از iterationها | thread کمکی و condition variable | با `regenerate_noises=true` واقعاً thread جدا وجود دارد |
| velocity smoothing | ۲۰ Hz؛ ۵۰ ms | `create_wall_timer` | فرمان قبلی را نگه می‌دارد؛ دریافت فرمان و tick دو کار متفاوت‌اند |
| collision monitor | با رسیدن فرمان smoothed | callback پیام‌محور | scanهای ذخیره‌شده و TF را هم می‌خواند؛ timer مستقل ۲۰ Hz برای محاسبه ندارد |
| tick درخت رفتار | فاصلهٔ اسمی ۱۰ ms | `WallRate` | کار orchestration است؛ والد تازهٔ همهٔ jobهای کنترل در هر tick نیست |
| behaviorهای recovery | چرخهٔ اسمی ۱۰ Hz هنگام فعال بودن behavior | action/worker مشروط | در دور موفق قبلی هیچ spin، backup یا wait اجرا نشده است |

وجود نام یک node در گراف به معنای اجرای الگوریتم آن در مأموریت نیست. SmootherServer آماده است، ولی در BT فعلی `SmoothPath` نداریم. WaypointFollower، RouteServer و DockingServer نیز مأموریت فعلیِ NavigateThroughPoses را انجام نمی‌دهند؛ مقدار frequency آن‌ها نباید به‌عنوان jobهای فعالِ کنترل وارد workload شود. اجزای lifecycle و TF و bridge، فعالیت‌های پشتیبان دارند و باید دامنهٔ اندازه‌گیری‌شان مشخص شود. GUI و rendering هزینهٔ host دارند، اما خودبه‌خود تسک CPU یک edge server فرضی نیستند.

پارامترهای مؤثر بر حجم کار از قبل مشخص‌اند: MPPI دارای batch_size=2000، time_steps=56 و iteration_count=1 است؛ model_dt=0.05 s گام پیش‌بینی مدل است، نه مدت اجرای CPU. AMCL حداکثر ۱۲۰ beam و ۵۰۰ تا ۲۰۰۰ particle دارد؛ trigger حرکتی آن ۰٫۰۵ m یا ۰٫۰۵ rad است. نقشهٔ محلی ۳×۳ m با resolution=0.05 m است؛ نقشهٔ ثابت ۱۰۰۹×۴۴۶ pixel دارد. planner از NavFn با use_astar=false استفاده می‌کند. این پارامترها ویژگی workload هستند و زمان اجرا را تعیین نمی‌کنند.

بازرسی چهارثانیه‌ایِ پیام LiDAR: ۳۶۰ range و ۳۶۰ intensity، یعنی ۲۸۸۰ byte فقط برای آرایه‌های float32، به‌علاوهٔ metadata و سربار serialization/transport. `scan_time` و `time_increment` هر دو صفر بودند؛ برای T منبع باید SDF و timestampها را ملاک قرار دهیم. اندازهٔ واقعی payload کل، serialization و transfer هنوز اندازه‌گیری نشده‌اند. گراف discovery عمق بعضی QoSها را صفر گزارش کرد؛ این را ظرفیت صفرِ صف یا نبود queue حساب نمی‌کنیم.

## ۲. داده‌های timing از پیش موجود و محدودیت آن‌ها

ComputePathThroughPoses در نتیجهٔ action فیلد `planning_time` دارد. در سورس این نسخه از اختلاف `node.now()` ابتدا و انتهای عملیات ساخته می‌شود؛ بنابراین elapsed timeِ clock نود است، هزینهٔ صرفِ CPU نیست و ممکن است انتظار برای costmap و کارهای جانبی را هم شامل شود. هنگام توقف `/clock` حتی می‌تواند معنای اندازه‌گیری مناسبی نداشته باشد. wrapper مأموریت ما این فیلدِ action داخلی را تاکنون ذخیره نکرده است.

Costmap در حالت DEBUG مدت `updateMap()` را با `ExecutionTimer` گزارش می‌کند؛ این timer از chrono high_resolution_clock استفاده می‌کند، نه thread CPU clock. MPPI یک اندازه‌گیری پشت macroِ compile-timeِ `BENCHMARK_TESTING` دارد؛ تعریف آن در سورس comment شده است و ROS parameter آماده برای فعال کردنش نیست. حتی آن بازه نیز lock wait را شامل می‌شود و visualization بعد از آن قرار دارد. هیچ‌کدام WCET یا بودجهٔ آمادهٔ C_i نمی‌دهند.

دور موفق موجود ۳۲۲٫۱۹۷ ثانیه wall time، ۱۳۶۳ نمونهٔ pose، ۷۶۵۳ نمونهٔ odometry velocity، ۲۰ transition هدف و نتیجهٔ موفقیت را ذخیره کرده است. این‌ها برای رفتار حرکتی و outcome مفیدند؛ CPU شروع/پایان هر کار، صف، costmap version و زمان آزادسازی job در آن‌ها نیست. بدون اجرای دوباره با ابزار مناسب، C_i دور قبلی را به‌طور معتبر استخراج نمی‌کنیم. تفاوت ساعت host و simulator، افت پیام یا sampling نیز مانع تبدیل مستقیم نرخ دریافت این نمونه‌ها به T رسمی است.

## ۳. مدل DAG که معنای نرم‌افزار را حفظ کند

گراف static نودها کافی نیست. کنترل و جهان فیزیکی feedback دارند، و نرخ‌های بخش‌های مختلف برابر نیستند. مدل مناسب‌تر، مجموعهٔ واحدهای کار با release ruleهای مختلف و DAGِ jobهای واقعی در یک افق زمانی محدود است. هر occurrence یک vertex مستقل دارد؛ state ورودیِ دور قبل از state خروجیِ دور جدید جدا می‌شود. چرخهٔ فیزیکی به scanها و jobهای آینده متصل می‌شود.

روابط مفهومی عبارت‌اند از:

```text
scan + odom/TF + state قبلی -> AMCL -> نسخهٔ جدید map-to-odom
scan + TF + observation buffer -> local/global costmap updates
global costmap + pose + اهداف باقی‌مانده -> planner -> path جدید
آخرین pose/odom + آخرین local costmap + آخرین path -> MPPI iteration
MPPI command -> دریافت در smoother -> tickِ smoother -> collision check -> Gazebo
```

این روابط در هر تکرار یک زنجیرهٔ اجباری از scan تازه تا planner تازه تا controller تازه نیستند. در حالت عادی controller بیست بار در ثانیه از آخرین path استفاده می‌کند، حتی اگر planner تقریباً یک بار در ثانیه اجرا شود. AMCL pose topic در snapshot فعلی subscriber ناوبری نداشت؛ ناوبری وضعیت را از TF/odom می‌گیرد. `/plan` نیز برای RViz بود؛ path کاربردی از نتیجهٔ action و goalِ FollowPath عبور می‌کند. بنابراین ساخت DAG صرفاً از topicهای مشهور اشتباه خواهد بود.

برای extraction علّی باید job_id، ورودیِ واقعاً خوانده‌شده، output/state version، action goal ID، timestamp و callback/thread شناخته شوند. shared memory، TF و costmap buffer باید دیده شوند. دو job که در baseline پشت سر هم اجرا شده‌اند لزوماً precedence ندارند؛ ترتیب اتفاقیِ اجرای baseline را نباید به DAG اضافه کنیم و schedulerهای دیگر را بی‌دلیل محدود کنیم. mutex و callback-group exclusion معمولاً resource constraints/blocking هستند و باید جدا از وابستگی داده مدل شوند.

در ComputePathThroughPoses یک DAG طبیعیِ محدود هم هست: برنامه‌ریزی segmentهای باقی‌مانده و اتصال آن‌ها. سورس segmentها را پشت سر هم اجرا می‌کند و نقطهٔ شروع segment بعدی را از آخرین poseِ خروجی قبلی می‌گیرد؛ مستقل فرض کردن و موازی کردن همهٔ segmentها معنای فعلی را حفظ نمی‌کند. تعداد آن‌ها با گذشتن از چک‌پوینت‌ها کم می‌شود، پس Cِ این action یک ثابتِ مستقل از ورودی نیست.

اگر تحلیل انتخابی فقط periodic DAG ثابت قبول کند، باید زیرگراف و contract آن صریحاً انتخاب شود؛ کل Nav2 را بدون تقریب یا تغییر semantics نمی‌توان همان مدل معرفی کرد. برای مرحلهٔ اول، استخراج multirate job graph با بخش‌های periodic، event-driven و conditional دقیق‌تر است. HI/LO، بودجه‌های mixed-criticality، WCET و deadline رسمی هنوز وجود ندارند؛ افزودن D_i بعداً این واقعیت را عوض نمی‌کند.

## ۴. اندازه‌گیری C و T

برای هر کارِ مورد بررسی release، eligibility/ready، dispatch/start، finish واقعی، commit/output availability و input timestamp را ثبت می‌کنیم. سه ساعت جدا نگه داشته می‌شوند: simulation time برای دنیای آزمایش، monotonic host time برای مدت elapsed و thread CPU time برای محاسبهٔ واقعاً مصرف‌شده. برای job تک-thread، اختلاف CLOCK_THREAD_CPUTIME_ID هزینهٔ CPU همان thread را می‌دهد؛ thread کمکیِ نویز MPPI و هر کار موازی دیگر جدا اندازه‌گیری و به مدل متصل می‌شوند.

انتظار executor/صف، انتظار قفل، انتظار پاسخ action/TF/costmap، sleepِ تنظیم نرخ و computation باید تفکیک شوند. مدت task برابر CPU demand نیست؛ response time نیز از release تا در دسترس شدن نتیجه تعریف می‌شود. اگر یک job چند thread دارد، جمع CPU demand با طول elapsed و ظرفیت لازم برای اجرای موازی یکسان نیست.

T اسمیِ اجزای periodic از configuration داریم؛ actual release/start فاصله‌ها، jitter و skipped/merged jobs نیاز به trace دارند. AMCL heavy update T ثابت ندارد و باید branch آن ثبت شود. حلقهٔ فعلی controller با اتمام computation به sleep/overrun می‌رسد: در اضافه‌بار الزاماً هر ۵۰ ms یک job تازه به صف اضافه نمی‌کند. مدل periodic با صف همهٔ occurrenceها، یا latest-only/drop، انتخاب معنایی است که باید برای همهٔ schedulerها یکسان باشد و با رفتار native مقایسه شود.

در این نصب tracetools 8.2.6 و LTTng کاربران موجودند و header tracetools tracing را غیرفعال اعلام نمی‌کند؛ ولی `ros2 trace` فعلاً نصب نیست و session daemon فعال نبود. فعال بودن tracepoints در header، تأییدِ عملیِ همهٔ eventهای runtime نیست. instrumentation عمومی برای callbackها مفید است، اما workerهای SimpleActionServer با std::async، costmap thread و iterationهای کنترل را کامل پوشش نمی‌دهد. tracepointهای مختصر در مرز این عملیات و ثبت dependency versions لازم است. اندازه‌گیری CPU داخل worker راهی برای مستقل بودن از دسترسی به kernel tracing در WSL است.

یک دور instrumented می‌تواند دادهٔ اولیه بدهد؛ برای distribution، p95/p99، بیشینهٔ مشاهده‌شده و شرایط سخت، تکرار و ورودی‌های متنوع لازم است. بیشینهٔ مشاهده‌شده را WCET اثبات‌شده نمی‌نامیم. اگر مقدار ثابت برای scheduler انتخاب کنیم، آن را empirical budget با دامنهٔ اندازه‌گیری/حاشیهٔ مشخص معرفی می‌کنیم. workload featureهایی مثل تعداد اهداف، طول path، اندازهٔ ناحیهٔ costmap تغییرکرده و تعداد particle نیز ذخیره می‌شوند.

## ۵. تبدیل به A15/A7

پس از profiling، به‌جای یک C واحد، برای هر نوع کار و hardware کلاس یک C_{i,h} یا تابع C_{i,h}(features) لازم است. نام CPU، GHz، تعداد core یا عددی مثل DMIPS به‌تنهایی زمانِ اجرای NavFn/MPPI/AMCL را از Intel تبدیل نمی‌کند؛ memory/cache، ISA و binary، vectorization، compiler، نوع عملیات و contention اثر دارند.

سه سطح معتبر با ادعاهای متفاوت داریم:

1. مدل انتزاعی: C_profile لپ‌تاپ و ضرایب صریحِ task-specific برای کلاس سریع/کند؛ مثلاً C_model(i,h)=alpha(i,h)*C_profile(i). این برای آزمایش scheduler قابل استفاده است، ولی alphaهای فرضی نتیجهٔ اندازه‌گیریِ A15/A7 نیستند. core count، قابلیت preemption، migration و overhead پارامترهای مدل‌اند و هنوز عددی برایشان انتخاب نشده است.
2. مدل calibrated: اجرای همان kernelهای محاسباتی با ورودی‌های نماینده روی SoC مشخصِ دارای A15/A7، ثبت clock/cache/memory و binary و fitِ مدل. هزینهٔ input/output snapshot و انتقال را جدا حساب می‌کنیم.
3. مدل microarchitecture: gem5 یا ابزار مشابه با تنظیم و اعتبارسنجی هدف. این مرحله پرهزینه‌تر است. انتخاب صرفِ یک نام CPU در QEMU یا استفاده از icount، timing معتبرِ A15/A7 تولید نمی‌کند؛ مستند QEMU صریحاً icount را cycle-accurate نمی‌داند.

A15/A7 در خانوادهٔ Armv7-A هستند؛ calibration روی دستگاه واقعی نیازمند binary مناسب همان target است، نه اجرای فایل amd64 لپ‌تاپ یا فرض امکان اجرای arm64. support matrix و toolchain target باید پیش از port بررسی شوند. برای مدل انتزاعیِ مورد نظر فعلی لازم نیست ROS/Gazebo را روی این پردازنده‌ها boot کنیم.

## ۶. توقف زمان و تحویل نتیجهٔ مدل‌شده

اصل ایده درست است، ولی pauseِ صرفِ Gazebo کافی نیست. در همین نسخه controller، costmap و BT با WallRate، smoother با wall timer و محدودکنندهٔ replanning با chronoِ host کار می‌کنند. use_sim_time ساعتِ node.now و timestamps را عوض می‌کند؛ همهٔ threadها، DDS، callbackهای آماده و timeoutهای host را متوقف نمی‌کند. اگر فقط physics متوقف شود، نرم‌افزار ممکن است بارها روی همان state محاسبه کند یا state داخلی‌اش را تغییر دهد.

معماری پیشنهادی:

- یک coordinator، زمان مشترکِ simulator و scheduler را اداره می‌کند. نگاشت دوره‌های host به این محورِ مشترک باید صریح باشد: جایگزین کردن ۵۰ ms از wall time با ۵۰ ms از simulation time هنگام real-time factor کمتر از یک، نسبت کنترل به زمان فیزیک را تغییر می‌دهد؛ reference مجازی و همهٔ سیاست‌ها باید همان قرارداد مشترک را داشته باشند. دوره‌های مربوط به مدل به همین زمان متصل می‌شوند؛ wall loops خارج از کنترل اجازهٔ release/compute خودسر ندارند.
- ورودی هر job در زمانِ dispatch مطابق سیاست sample/hold انتخاب و نسخه‌گذاری می‌شود. worker همان الگوریتم موجود را روی لپ‌تاپ اجرا می‌کند. اگر کند باشد، coordinator پیشروی simulation time را تا آماده شدن محاسبه متوقف می‌کند؛ wall time محاسبه وارد C مدل‌شده نمی‌شود.
- output و state delta خصوصی می‌مانند. scheduler با C_{i,h}، صف، core، preemption و هزینه‌های خودش، زمان پایان و سپس network delivery را تعیین می‌کند.
- Gazebo در طول فاصلهٔ مدل‌شده پیش می‌رود، با فرمان قبلی/نتایج دیگری که رسیده‌اند. sensor releaseها، task releaseها، completionها و transmissionها میان این فاصله پردازش می‌شوند. نتیجه فقط در زمان availability به ربات و successorها تحویل می‌شود.
- پیشروی در گام‌های کنترل‌شده با barrier انجام می‌شود؛ یک jump بزرگِ /clock جای physics integration و سنسورهای بین راه را نمی‌گیرد. service فعلی `/world/monaco/control` از نوع WorldControl/Boolean با introspection تأیید شد، ولی pause/step در این بررسی اجرا نشد. physics step فعلی ۰٫۰۰۳ s است و باید خطای quantization زمان events را در طراحی لحاظ کنیم.

مثال صرفاً توضیحی: job در t_sim=10.00 آزاد و به A7 مدل‌شده dispatch می‌شود؛ C=1.40 s فرض شده است و لپ‌تاپ آن را در ۰٫۰۳ s wall محاسبه می‌کند. نتیجه آماده است ولی مخفی می‌ماند. simulator باید رخدادهای t=10.05،10.10 و ... را طبق release contract پردازش کند؛ فقط در t=11.40 computation تمام‌شده محسوب می‌شود، و با مثلاً ۰٫۰۲ s ارتباط، delivery برابر 11.42 می‌شود. اگر preemption یا صف وجود داشته باشد، 11.40 نیز تغییر می‌کند. عددهای این مثال، دادهٔ اندازه‌گیری‌شده نیستند.

اگر همان نوع job stateful غیرقابل reentrant باشد، نوبت بعد نمی‌تواند stateِ پایانِ نوبت اول را پیش از زمان پایان مدل‌شده مصرف کند. کافی نیست فقط آخرین topic را delay کنیم: AMCL particle state، MPPI warm-start/control sequence/noise state و costmap shared memory نیز باید sequencing یا commit کنترل‌شده داشته باشند. برای مدل preemptive می‌توان computationِ host را اتمیک انجام داد و زمان سرویس را مجازی قطع کرد، تنها اگر intermediate side effectها بیرون درز نکنند و release stateful صحیح بماند؛ این ادعای preemption واقعیِ binary روی A7 نیست.

## مرزهای اجرایی پیشنهادی و میزان تغییر لازم

| جزء | مرز عملی برای adapter | آنچه باید کنترل شود |
|---|---|---|
| planner | ورودی action و getPlan/computePlanThroughPoses | snapshot نقشه/pose؛ publicationِ `/plan` و action result هر دو؛ فقط تأخیر در result کافی نیست |
| controller | یک computeAndPublishVelocity و فراخوانی plugin آماده | release حلقه، ورودیِ زمان dispatch، command publication، دسترسی به costmap و stateِ MPPI |
| AMCL | laserReceived با ورودی scan/TF مشخص | ترتیب statefulِ particle filter و انتشار map-to-odom/pose؛ آماده شدن TF برای scanهای بعدی |
| costmap | updateMap و commitِ نسخهٔ جدید | تغییر حافظهٔ مشترک نباید قبل از پایان مدل‌شده دیده شود؛ publication تنها مصرف‌کنندهٔ آن نیست |
| smoother/collision | callback دریافت، tickِ smoother و processِ collision | اتصال release به ساعت مدل، cached inputs، late/drop rules و تحویل فرمان به Gazebo |

برای planner معمولاً interface آمادهٔ action مرز بیرونی روشن‌تری می‌دهد. مدل کامل همهٔ اجزا به adapter یا patch محدود در اسکلتِ اجرا/انتشار و مدیریت state نیاز دارد؛ الگوریتم‌های NavFn، MPPI و AMCL می‌توانند همان الگوریتم‌های موجود بمانند. این اندازه‌گیری یا شبیه‌سازیِ timing با یک YAML یا pause ساده فعال نمی‌شود. اگر state خصوصیِ یک worker زودتر روی host تغییر کند، باید آن worker و هر خوانندهٔ state تا commitِ منطقی اجازهٔ اجرای نوبتِ بعد نداشته باشند؛ در صورت نیاز به concurrency بیشتر، نسخه‌گذاری/snapshot واقعی لازم می‌شود.

## ۷. overhead، ارتباط و رفتار قابل مشاهده

برای مدل تک-core و بدون preemption، زمان completion از dispatch به‌علاوهٔ C و هزینه‌های صریحِ همان مدل به دست می‌آید؛ queue delay پیش از dispatch و transfer پس از completion جدا هستند. برای چند core یا preemption، event scheduler با remaining work و resource constraints لازم است، نه یک sleep ثابت برای هر پیام. overheadِ تصمیم‌گیری scheduler، context switch، migration، serialization، input snapshot و commit باید مشخص شود و دوبار شمرده نشود. overhead واقعیِ coordinator روی لپ‌تاپ از overheadِ منتسب به پردازندهٔ مدل‌شده جدا گزارش می‌شود؛ صرفِ اجرای scheduler در wall time نباید به‌طور نامعلوم نتیجه را آلوده کند.

گراف فعلی از DDS/action و shared memory استفاده می‌کند؛ offloadِ costmap یا MPPI به server مستقل مستلزم interface و snapshot واقعی است. وضعیت resource، link و cache affinity باید در مدل تعریف شود. edge carهای آبی اکنون فقط مکان‌های نمایشی‌اند و پردازنده/worker ندارند.

تأخیرها باید در خودِ زمان فیزیک شبیه‌سازی اثر بگذارند تا فیلم معتبر باشد. در جریان ۱٫۴ s تأخیر، ماشین با فرمان قبلی حرکت می‌کند یا به علت مکانیزم ایمنی می‌ایستد؛ نباید این بازه از حرکت حذف شود. timeoutهای موجود از قبل داریم: velocity_timeout=1.0 s، collision source_timeout=1.0 s، costmap_update_timeout کنترلر ۰٫۳ s و planner ۱ s، progress allowance=10 s. این‌ها formal D_i نیستند، ولی اثرِ نتایج دیررس را محدود می‌کنند. timestamp تولید/ورودی قدیمی را با timestamp تازهٔ delivery جایگزین نمی‌کنیم تا کهنگی پنهان نشود.

یک لاگ ثابت برای مقایسهٔ offlineِ schedulerها مفید است؛ برای مقایسهٔ رفتاریِ closed-loop، releaseها، branchها و workload ممکن است در پاسخ به تأخیرها عوض شوند. باید همان world، sensor parameters، الگوریتم، seed/reproducibility protocol و release/drop contracts ثابت باشند و از وضعیت زندهٔ هر run محاسبه کنیم. بازپخش همهٔ ورودی‌های دور baseline همراه با ادعای حرکتِ closed-loop صحیح نیست. در ساعت مجازی نتیجه روی ربات شبیه‌سازی‌شده معتبر می‌شود؛ مدتِ انتظار انسانی ممکن است بلندتر از مدت simulation باشد. wall-clock سختِ ۱× برای ربات فیزیکی با لپ‌تاپ کندتر از target تضمین‌پذیر نیست؛ فیلم خروجی را بعداً می‌توان بر اساس simulation timestamps در سرعت ۱× نمایش داد.

## ۸. ابزارها و مرحلهٔ بعدی پیشنهادی

ros2_tracing پایهٔ instrumentation و اتصال callback/messageهاست؛ پروژهٔ Trace-enabled Timing Model Synthesis for ROS2-based Autonomous Applications نیز ساخت DAG علّیِ زمان‌دار از ROS/OS trace را بررسی کرده و مرجع مرتبطی است، نه ابزار آمادهٔ تأییدشده برای threadهای Nav2 ما.

rslcpp یک مرجع نزدیک برای تفکیک computation host از simulation time و delayed publication است. روش مقاله به idle شدن حلقهٔ کنترل‌شده و callbackهای بدون blocking وابسته است؛ برای delay modeling و wall timers نیاز به rclcpp اصلاح‌شده مطرح می‌کند. با توجه به workerهای مستقل، wall loops، waits و state مشترک Nav2، سازگاری آن با این نصب هنوز آزمایش نشده و جایگزین آمادهٔ scheduler ناهمگن ما فرض نمی‌شود. این ایدهٔ عمومیِ تأخیر مصنوعی سابقه دارد؛ سهم پژوهشیِ زمان‌بندی باید از سیاست و ارزیابی مشخص خودمان بیاید.

ترتیب مناسب کار پس از تصمیم مشترک:

1. انتخاب scope و job boundaries واقعی؛ ثبت اینکه outer iteration شامل چه computation و visualization است و helper thread چه جایگاهی دارد.
2. اجرای مجدد همان مسیر با trace از قبل از startup؛ release/CPU/elapsed/locks/input-state versions را ثبت کنیم. عدم تغییر رفتار و overhead tracer را با اجرای عادی مقایسه کنیم.
3. ساخت جدول empirical C_i و nominal/observed T_i و DAGهای jobهای رخ‌داده؛ D_i و criticality را باز بگذاریم. نواقص trace را به مدل تبدیل نکنیم.
4. انتخاب مدل ظرفیت/کالیبراسیون A15/A7 و contract صف/late job؛ سپس یک proof of concept محدود برای یک job واقعی بسازیم و barrier/commit را بررسی کنیم.
5. پس از تأیید این مرز، همهٔ workerها و دوره‌ها را به زمان coordinator ببریم و closed-loop scheduling comparison را اجرا کنیم. سیاست scheduler، deadlineها، CPU/core count و شبکه هنوز تصمیم‌گیری نشده‌اند.

## منابع اولیه و قابل بازبینی

- [ControllerServer 1.3.13](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_controller/src/controller_server.cpp): worker iteration، WallRate، costmap wait و publish.
- [Costmap2DROS 1.3.13](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_costmap_2d/src/costmap_2d_ros.cpp): mapUpdateLoop و DEBUG timing.
- [PlannerServer 1.3.13](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_planner/src/planner_server.cpp): segment sequencing و planning_time.
- [AMCL 1.3.13](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_amcl/src/amcl_node.cpp): scan/TF و shouldUpdateFilter.
- [VelocitySmoother 1.3.13](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_velocity_smoother/src/velocity_smoother.cpp)، [BT Engine](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_behavior_tree/src/behavior_tree_engine.cpp)، [RateController](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_behavior_tree/plugins/decorator/rate_controller.cpp): ساعت‌های host.
- [MPPI controller](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_mppi_controller/src/controller.cpp)، [optimizer](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_mppi_controller/src/optimizer.cpp)، [noise generator](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_mppi_controller/src/noise_generator.cpp): locks، state و thread کمکی.
- [ROS Clock and Time](https://design.ros2.org/articles/clock_and_time.html)، [Gazebo Harmonic pause/control](https://gazebosim.org/api/sim/8/pause_run_simulation.html).
- [ros2_tracing Jazzy](https://github.com/ros2/ros2_tracing/tree/jazzy)، [Trace-enabled Timing Model Synthesis](https://arxiv.org/abs/2311.13333).
- [RSLCPP paper](https://arxiv.org/html/2601.07052v1)، [rslcpp repository](https://github.com/TUMFTM/rslcpp).
- [gem5 ARM configurations/DVFS](https://www.gem5.org/documentation/learning_gem5/part2/arm_dvfs_support/)، [QEMU icount limitation](https://www.qemu.org/docs/master/devel/tcg-icount.html).
- [Arm toolchain architecture table](https://developer.arm.com/Tools%20and%20Software/Arm%20Toolchain%20for%20Embedded)، [ROS target platforms REP-2000](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst).

اطلاعات پیکربندیِ همین پروژه و snapshot گراف در `docs/evidence/task-abstraction-inventory.json` آمده است. nullها عمداً ناشناخته‌اند و اندازه‌گیری نشده‌اند.
