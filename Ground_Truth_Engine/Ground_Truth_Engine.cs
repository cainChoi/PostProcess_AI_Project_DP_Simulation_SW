using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;
using RadarSim.Core;
using System.Numerics;


namespace Ground_Truth_Engine
{
    public class Ground_Truth_Engine
    {
        private ITrajectoryModel activeTargetModule;

        /// <summary>
        /// Loads the target module.(플러그인 구조)
        /// </summary>
        /// <param name="dllPath">The DLL path.</param>
        public void LoadTargetModule(string dllPath, ITargetParameters parameters)
        {
            // 1. 선택된 DLL 파일을 런타임에 불러옴
            // 예: "C:\Plugins\BallisticModule.dll"
            Assembly pluginAssembly = Assembly.LoadFrom(dllPath);

            // 2. DLL 내부를 뒤져서 'ITrajectoryModel' 약속을 지킨
            //    클래스를 찾아냄
            Type moduleType = pluginAssembly.GetTypes()
                .FirstOrDefault(t => typeof(ITrajectoryModel).IsAssignableFrom(t));

            if (moduleType != null)
            {
                // 3. 해당 클래스의 인스턴스를 생성
                //    (WPF는 이게 'BallisticModule'인지 'DroneModule'인지 모름)
                activeTargetModule = (ITrajectoryModel)Activator.CreateInstance(moduleType);

                activeTargetModule.Initialize(parameters);
            }
        }

        // 5. 시뮬레이션 루프
        public void RunSimulationStep(double dt)
        {
            // WPF는 그냥 '약속된' Update 기능만 호출함
            activeTargetModule?.Update(dt);
            Vector3 currentPos = activeTargetModule.Position;
            // ...
        }
    }
}
