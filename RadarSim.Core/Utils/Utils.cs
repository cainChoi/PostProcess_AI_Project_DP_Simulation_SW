using System;
using System.Collections; // IEnumerable
using System.Collections.Generic;
using System.Linq;
using System.Reflection; // Reflection

namespace RadarSim.Core
{
    /// <summary>
    /// 리플렉션을 사용해 객체 리스트에서 중첩된 속성 값을
    /// 문자열 경로로 추출하는 유틸리티 클래스입니다.
    /// </summary>
    public static class PropertyExtractor
    {
        /// <summary>
        /// 리스트의 각 항목에서 "Some.Nested.Property" 같은
        /// 문자열 경로를 따라 값을 추출합니다.
        /// </summary>
        /// <param name="sourceList">데이터가 담긴 원본 리스트</param>
        /// <param name="propertyPath">추출할 속성의 경로 (예: "TrajectoryLog.Position.Y")</param>
        /// <returns>추출된 값들이 담긴 리스트. 실패 시 빈 리스트.</returns>
        public static List<object> ExtractNestedProperty(IEnumerable sourceList, string propertyPath)
        {
            var results = new List<object>();

            // 1. 경로 분리: "TrajectoryLog.Position.Y" -> ["TrajectoryLog", "Position", "Y"]
            string[] propertyNames = propertyPath.Split('.');

            if (sourceList == null || !propertyNames.Any())
            {
                return results;
            }

            // ⬇️ 1. 바인딩 플래그 정의
            //    (Public, NonPublic, Instance 멤버를 모두 검색)
            BindingFlags flags = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;

            try
            {
                foreach (var item in sourceList)
                {
                    object currentValue = item;
                    foreach (string name in propertyNames)
                    {
                        if (currentValue == null) break;

                        Type currentType = currentValue.GetType();

                        // ⬇️ 2. Property(속성) 먼저 검색
                        PropertyInfo propInfo = currentType.GetProperty(name, flags);
                        if (propInfo != null)
                        {
                            currentValue = propInfo.GetValue(currentValue);
                        }
                        else
                        {
                            // ⬇️ 3. Property가 없으면 Field(필드) 검색
                            FieldInfo fieldInfo = currentType.GetField(name, flags);
                            if (fieldInfo != null)
                            {
                                currentValue = fieldInfo.GetValue(currentValue);
                            }
                            else
                            {
                                // 둘 다 없으면 오류
                                throw new ArgumentException($"'{name}' 속성 또는 필드를 '{currentType.Name}'에서 찾을 수 없습니다.");
                            }
                        }
                    }
                    results.Add(currentValue);
                }
            }
            catch (Exception ex)
            {
                // (오류 처리: 예: WPF의 MessageBox로 오류 표시)
                System.Diagnostics.Debug.Print($"속성 추출 오류: {ex.Message}");
                return new List<object>(); // 오류 시 빈 리스트 반환
            }

            return results;
        }
    }
}
