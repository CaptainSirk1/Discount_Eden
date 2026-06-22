[System.Serializable]
public class ClimateState
{
    
    public AtmosphericData atmosphere;
    public float globalMoisture;
    public float averageTemp;
    public float actualSeaLevel;

    public ClimateState(ClimateSettings settings)
    {
        
        // Copy starting values from the asset — we never touch the asset again.
        atmosphere = new AtmosphericData
        {
            nitrogen = settings.atmosphere.nitrogen,
            carbonDioxide = settings.atmosphere.carbonDioxide,
            methane = settings.atmosphere.methane,
            oxygen = settings.atmosphere.oxygen,
            traceElements = settings.atmosphere.traceElements
        };
        globalMoisture = settings.globalMoisture;
        averageTemp = settings.averageTemp;
        RecalculateDerivedValues();
    }

    // Call this whenever a game event changes globalMoisture or averageTemp.
    public void RecalculateDerivedValues()
    {
        if (averageTemp <= 1.0f)
        {
            actualSeaLevel = globalMoisture * UnityEngine.Mathf.Pow(
                UnityEngine.Mathf.Clamp01(averageTemp), 0.5f);
        }
        else
        {
            float excess = averageTemp - 1.0f;
            actualSeaLevel = globalMoisture / (1.0f + (excess * excess * 10.0f));
        }

        float ambientEvap = actualSeaLevel * UnityEngine.Mathf.Min(averageTemp, 1.0f) * 0.2f;
        float boiledOffSteam = UnityEngine.Mathf.Max(0, globalMoisture - actualSeaLevel);
        float rawVapor = ambientEvap + boiledOffSteam;
        float vaporPercentage = rawVapor / (0.1f + rawVapor);

        atmosphere.BalanceDryRecipe();
        atmosphere.CalculateFinalState(UnityEngine.Mathf.Min(vaporPercentage, 0.99f));
    }
}