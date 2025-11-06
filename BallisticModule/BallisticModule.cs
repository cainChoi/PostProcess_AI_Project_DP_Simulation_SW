using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RadarSim.Core.Trajectory;
using System.Numerics;

namespace BallisticModule
{
    public class BallisticModule : ITrajectoryModel
    {
        /// <summary>
        /// (항력 계수)
        /// </summary>
        private double dragCoefficient;
        /// <summary>
        /// (질량 - F=ma 계산용, 파라미터로 받을 수 있음)
        /// </summary>
        private double mass;
        /// <summary>
        /// (항력 계산용 단면적, 파라미터로 받을 수 있음)
        /// </summary>
        private double crossSectionalArea;

        private double maxMicroVelocity; // (내부 변수로 사용)
        private double spinRate_Hz;
        private double radius_m;


        /// <summary>
        /// 현재 상태 저장
        /// </summary>
        private Vector3 currentPosition;

        /// <summary>
        /// 현재 상태 저장
        /// </summary>
        private Vector3 currentVelocity;

        /// <summary>
        /// 현재 상태 저장
        /// </summary>
        private Quaternion currentOrientation;

        /// <summary>
        /// RCS 룩업 테이블.
        /// Key = 각도(Degrees, 0=정면), Value = RCS(m^2)
        /// </summary>
        private SortedList<double, double> rcsLookupTable;

        public bool IsActive { get; private set; }

        public SnapShot GetSnapshot()
        {
            return new SnapShot(currentPosition, currentVelocity, currentOrientation, IsActive);
        }


        // 3. 인터페이스의 모든 기능을 실제로 구현
        public void Initialize(ITargetParameters parameters)
        {
            //입력된 parameters를 BallisticParameters로** 형 변환(casting) * *합니다.
            //BallisticParameters에서 초기 위치, 속도, 항력 계수 등을 읽어와 내부 멤버 변수를 초기화합니다.
            
            // WPF가 전달한 ITargetParameters를
            // BallisticParameters로 안전하게 형 변환!
            if (parameters is TargetParameters ballisticParams)
            {
                // UI에서 입력한 구체적인 값들을 사용
                this.dragCoefficient = ballisticParams.DragCoefficient;
                this.mass = ballisticParams.Mass;
                this.crossSectionalArea = ballisticParams.CrossSectionalArea;

                //currentPosition, currentVelocity, currentOrientation을 설정합니다.
                // 초기 속도 벡터 계산
                this.currentVelocity = InitializeVelocityVector(
                    ballisticParams.InitialSpeed,
                    ballisticParams.LaunchAzimuth,
                    ballisticParams.LaunchElevation
                );

                this.currentPosition = ballisticParams.InitialPosition;


                this.currentOrientation = Quaternion.CreateFromRotationMatrix(Matrix4x4.CreateLookAt(Vector3.Zero, this.currentVelocity, Vector3.UnitY));


                //RCS 룩업 테이블(LUT)을 이 시점에서 로드하거나 생성합니다. (예: rcsLookupTable = new ...)
                this.rcsLookupTable = new SortedList<double, double>()
                {
                    // 각도(도), RCS(m^2)
                    { 0.0,   0.01 }, // 0도 (정면)
                    { 30.0,  0.05 },
                    { 60.0,  0.5 },
                    { 90.0,  1.0 },  // 90도 (측면, 최대)
                    { 120.0, 0.5 },
                    { 150.0, 0.05 },
                    { 180.0, 0.02 }  // 180도 (후면)
                };


                this.spinRate_Hz = ballisticParams.SpinRate_Hz;

                // ⬇️ 2. 단면적(Area)에서 반경(Radius) 계산
                this.radius_m = Math.Sqrt(ballisticParams.CrossSectionalArea / Math.PI);

                // ⬇️ 3. 최대 미세 속도(접선 속도) 계산
                // V_tan = 2 * pi * r * f_spin
                this.maxMicroVelocity = 2.0 * Math.PI * this.radius_m * this.spinRate_Hz;

                this.IsActive = true;
            }
            else
            {
                // 잘못된 파라미터가 들어온 경우 예외 처리
                throw new ArgumentException("Invalid parameter type for BallisticModule");
            }
        }

        /// <summary>
        /// 스칼라 속력과 발사 각도를 기반으로
        /// 3D 초기 속도 벡터 (currentVelocity)를 계산하고 설정합니다.
        /// (Y축을 'Up'으로, +Z축을 0도 방위각 'North'로 가정)
        /// </summary>
        /// <param name="speed">초기 스칼라 속력 (m/s)</param>
        /// <param name="azimuthDeg">방위각 (degrees, 0 = +Z)</param>
        /// <param name="elevationDeg">고각 (degrees)</param>
        /// <returns></returns>
        private Vector3 InitializeVelocityVector(double speed, double azimuthDeg, double elevationDeg)
        {
            // 1. Degree -> Radian 변환
            // C#의 삼각함수는 라디안을 사용합니다.
            double azRad = Math.PI / 180.0 * azimuthDeg;
            double elRad = Math.PI / 180.0 * elevationDeg;

            // 2. 구면 좌표계 -> 직교 좌표계 변환
            // (좌표계 가정: Y = Up, Z = North/Forward, X = East/Right)

            // 수직 성분 (Y)
            float vy = (float)(speed * Math.Sin(elRad));

            // 수평면 투영 속력
            double horizontalSpeed = speed * Math.Cos(elRad);

            // 수평 성분 (X, Z)
            float vx = (float)(horizontalSpeed * Math.Sin(azRad));
            float vz = (float)(horizontalSpeed * Math.Cos(azRad));

            // 3. 최종 속도 벡터를 멤버 변수에 설정
            return new Vector3(vx, vy, vz);
            
        }

        public void Update(double timeDelta)
        {

            if (!this.IsActive)
            {
                return;
            }

            //1.공기 밀도 계산: rho = GetAirDensity(currentPosition.Y)(고도에 따른 공기 밀도 함수, 단순화를 위해 상수로 시작해도 됨)
            double rho = RadarSim.Core.PhysicsUtils.GetAirDensity(this.currentPosition.Y);

            //2.항력(Drag Force) 계산:

            // 속도의 크기
            double speed = currentVelocity.Length();

            // 항력의 크기
            double dragMagnitude = 0.5 * rho * speed * speed * this.dragCoefficient * this.crossSectionalArea;

            // 항력의 방향
            var dragDirection = -Vector3.Normalize(currentVelocity);

            // (speed가 0일 때 Normalize는 NaN을 반환할 수 있으므로 예외 처리)
            if (speed == 0)
            {
                dragDirection = Vector3.Zero;
            }

            // 항력 벡터
            var dragForce = dragDirection * (float)dragMagnitude;

            // 중력(Gravity Force) 계산
            var gravityForce = new Vector3(0, (float)(-9.81f * mass), 0);

            // 총 힘(Total Force) 계산
            var totalForce = dragForce + gravityForce;

            // 가속도(Acceleration) 계산:
            var acceleration = totalForce / (float)mass;

            //6.상태 업데이트(적분)
            currentVelocity = currentVelocity + acceleration * (float)timeDelta;
            currentPosition = currentPosition + currentVelocity * (float)timeDelta;

            if (this.currentPosition.Y <= 0)
            {
                this.IsActive = false; // ⬇️ 해수면에 닿으면 비활성화
            }

            //// 7. 디버그 출력
            //System.Diagnostics.Debug.Print(
            //    $"Pos.Y: {currentPosition.Y:F2} m | " +
            //    $"Vel.Y: {currentVelocity.Y:F2} m/s | " +
            //    $"Acc.Y: {acceleration.Y:F2} m/s^2"
            //);

            //8.자세 업데이트
            currentOrientation = UpdateOrientation(this.currentVelocity);
        }


        /// <summary>
        /// 현재 속도(currentVelocity)를 기반으로
        /// 표적의 자세(currentOrientation)를 업데이트합니다.
        /// </summary>
        private Quaternion UpdateOrientation(Vector3 velocity)
        {
            // 속도가 0에 가까우면 (방향성이 없으면)
            // 계산 오류(NaN)를 방지하기 위해 이전 자세를 유지합니다.
            if (velocity.LengthSquared() < 0.0001f)
            {
                return this.currentOrientation; // 이전 방향 유지
            }

            // CreateLookAt: System.Numerics에서 방향을 다루는 가장 강력하고 안정적인 함수. 
            // 포탄이 수직으로 상승하거나 하강할 때(Gimbal Lock)도 Vector3.UnitY("월드 Up")를 기준으로 방향을 올바르게 계산해 줍니다.
            // 1. "LookAt" 뷰 매트릭스 생성
            //    - (0,0,0)에서 currentVelocity 방향을 바라보고,
            //    - 머리 위(Up)는 (0,1,0) (World Up)을 기준으로 합니다.
            Matrix4x4 lookAtMatrix = Matrix4x4.CreateLookAt(
                Vector3.Zero,           // '카메라' 위치 (원점)
                velocity,   // '바라볼 대상' (속도 방향)
                Vector3.UnitY           // '월드 Up' 벡터
            );

            // Matrix4x4.Invert: CreateLookAt은 세상을 "바라보는" 뷰 매트릭스를 만듬. 
            // 우리가 필요한 것은 모델을 세상에 "배치하는" 월드 매트릭스(혹은 회전)입니다. 이 둘은 서로 역관계(Inverse)에 있으므로, Invert가 필요
            // 2. 뷰 매트릭스 -> 오브젝트 회전 매트릭스로 변환 (Invert)
            //    CreateLookAt은 '뷰' 매트릭스(World->View)를 만듭니다.
            //    우리는 '오브젝트' 매트릭스(Local->World)가 필요하므로,
            //    이를 뒤집어(Invert) 줍니다.
            Matrix4x4.Invert(lookAtMatrix, out Matrix4x4 objectRotationMatrix);

            // 3. 회전 매트릭스 -> 쿼터니언으로 변환하여 저장
            return Quaternion.CreateFromRotationMatrix(objectRotationMatrix);
        }

        public Vector3 Position { get { return currentPosition; } }

        public Vector3 Velocity { get { return currentVelocity; } }

        public Quaternion Orientation { get { return currentOrientation; } }

        /// <summary>
        /// 특정 '입사각 벡터'에 대한 RCS 값을 반환합니다.
        /// </summary>
        /// <param name="aspectAngleVector">
        /// 레이더가 표적을 바라보는 방향 벡터 (표적의 로컬 좌표계 기준)
        /// </param>
        /// <returns>RCS 값 [m^2]</returns>
        public double GetRCS(Vector3 aspectAngleVector)
        {
            // 1. 표적의 "정면" 방향을 로컬 좌표계 기준으로 정의합니다.
            //    (일반적으로 속도 방향인 +X 축을 정면으로 모델링합니다)
            Vector3 forwardVector = Vector3.UnitX;

            // 2. 두 벡터의 내적(Dot Product)을 사용해 각도의 코사인 값을 구합니다.
            //    (벡터는 정규화(Normalize)되어 있다고 가정하거나, 정규화 수행)
            float dotProduct = Vector3.Dot(
                forwardVector,
                Vector3.Normalize(aspectAngleVector)
            );

            // 3. Acos 계산 오류 방지를 위해 값을 [-1, 1] 범위로 고정(Clamp)
            dotProduct = Math.Max(-1.0f, Math.Min(1.0f, dotProduct));

            // 4. 라디안(Radian) 각도를 계산하고 도(Degree)로 변환
            double angleRad = Math.Acos(dotProduct);
            double angleDeg = angleRad * (180.0 / Math.PI);

            // 5. 헬퍼 함수를 사용해 LUT에서 보간된 RCS 값을 조회
            return InterpolateRcsLut(angleDeg);
        }

        /// <summary>
        /// RCS 룩업 테이블(LUT)에서 특정 각도의 RCS 값을
        /// 선형 보간하여 반환합니다.
        /// </summary>
        /// <param name="angleDeg">조회할 각도 (Degrees)</param>
        /// <returns>보간된 RCS 값 (m^2)</returns>
        private double InterpolateRcsLut(double angleDeg)
        {
            // 테이블이 비어있으면 0 반환
            if (this.rcsLookupTable == null || this.rcsLookupTable.Count == 0)
            {
                return 0.0; // 또는 기본값
            }

            // 1. 테이블 경계값 확인 (하한)
            //    입력된 각도가 테이블의 최소 각도보다 작거나 같으면,
            //    첫 번째 RCS 값을 그대로 반환합니다.
            if (angleDeg <= this.rcsLookupTable.Keys[0])
            {
                return this.rcsLookupTable.Values[0];
            }

            // 2. 테이블 경계값 확인 (상한)
            //    입력된 각도가 테이블의 최대 각도보다 크거나 같으면,
            //    마지막 RCS 값을 그대로 반환합니다.
            int lastIndex = this.rcsLookupTable.Count - 1;
            if (angleDeg >= this.rcsLookupTable.Keys[lastIndex])
            {
                return this.rcsLookupTable.Values[lastIndex];
            }

            // 3. 보간할 두 지점(각도) 찾기
            //    입력 각도(angleDeg)보다 바로 작거나 같은 키(lowerAngle)를 찾습니다.
            int i = 0;
            while (i < lastIndex && this.rcsLookupTable.Keys[i + 1] < angleDeg)
            {
                i++;
            }

            // 4. 보간에 사용할 값들 정의
            //    i = 하한 인덱스, i+1 = 상한 인덱스
            double lowerAngle = this.rcsLookupTable.Keys[i];
            double upperAngle = this.rcsLookupTable.Keys[i + 1];
            double lowerRCS = this.rcsLookupTable.Values[i];
            double upperRCS = this.rcsLookupTable.Values[i + 1];

            // (방어 코드: 두 각도가 같으면 나눗셈 오류 방지)
            if (lowerAngle == upperAngle)
            {
                return lowerRCS;
            }

            // 5. 보간 비율(t) 계산
            //    입력 각도가 두 각도 사이 어디쯤(0.0 ~ 1.0)에 위치하는지 계산
            double t = (angleDeg - lowerAngle) / (upperAngle - lowerAngle);

            // 6. 선형 보간 (Lerp)
            //    return lowerRCS * (1.0 - t) + upperRCS * t;
            return lowerRCS + t * (upperRCS - lowerRCS);
        }

        public double[] GetMicroDopplerPhaseNoise(int numSamples,
                                              double timeStep,
                                              double carrierFrequency,
                                              Random rand)
        {
            double[] phaseNoiseArray = new double[numSamples];

            // (1) 최대 미세 도플러 주파수 계산
            const double C = 299792458.0;
            double maxMicroDoppler_Hz = (2 * this.maxMicroVelocity * carrierFrequency) / C;

            // (2) 1 샘플당 최대 위상 변화량 (rad/sample)
            double maxPhaseStep = (2.0 * Math.PI * maxMicroDoppler_Hz) * timeStep;

            // (3) Random Walk 누적
            double phase_noise = 0.0;
            for (int s = 0; s < numSamples; s++)
            {
                phase_noise += (rand.NextDouble() * 2.0 - 1.0) * maxPhaseStep;
                phaseNoiseArray[s] = phase_noise;
            }

            return phaseNoiseArray;
        }
    }
}
