// Unlit vertex-colour shader for the tectonics preview mesh.
// Plain CG pass with no LightMode tag, so it renders in both the built-in pipeline and URP.
Shader "PlanetGen/TectonicPreview"
{
    Properties
    {
        [Enum(UnityEngine.Rendering.CullMode)] _Cull ("Cull", Float) = 2
    }
    SubShader
    {
        Tags { "RenderType" = "Opaque" "Queue" = "Geometry" }
        Cull [_Cull]
        ZWrite On

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata { float4 vertex : POSITION; fixed4 color : COLOR; };
            struct v2f { float4 pos : SV_POSITION; half4 color : COLOR; };

            v2f vert (appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.color = v.color;
                #if !defined(UNITY_COLORSPACE_GAMMA)
                o.color.rgb = GammaToLinearSpace(o.color.rgb);   // match gizmo colours in Linear projects
                #endif
                return o;
            }

            half4 frag (v2f i) : SV_Target { return i.color; }
            ENDCG
        }
    }
}
